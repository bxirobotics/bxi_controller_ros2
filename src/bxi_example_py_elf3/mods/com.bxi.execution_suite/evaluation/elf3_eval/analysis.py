"""Segmented causal comparisons. No future target or cross-boundary interpolation."""
from collections import Counter
from dataclasses import dataclass, asdict
import json
from pathlib import Path
import numpy as np
from .contract import BODY, GROUPS, vector, dump
from .math_core import causal_indices, metrics
from .lag import estimate
from .storage import open_db, load_epoch

@dataclass
class Options:
    command_age_ms: float | None = None
    model_age_ms: float | None = None
    feedback_gap_ms: float | None = None
    lag_max_ms: float = 250.
    lag_step_ms: float = 5.
    include_phases: tuple = ('active',)
    model: str | None = None
    start: float | None = None
    end: float | None = None
    sonic_reference: str | None = None
    acceptance_profile: str | None = None
    test_state: str | None = None

def timing(rows):
    t = np.array([r['t'] for r in rows if r.get('time_valid',True)])
    dt = np.diff(t); dt=dt[dt>0]
    return dict(samples=len(t), observed_hz=float((len(t)-1)/(t[-1]-t[0])) if len(t)>1 else None,
                median_period_s=float(np.median(dt)) if len(dt) else None,
                max_interval_s=float(np.max(dt)) if len(dt) else None,
                gaps_over_3_median_intervals=int((dt>3*np.median(dt)).sum()) if len(dt) else 0)

def segments(events):
    boundaries = {}; identity = None; mode = None; command_phase = None
    for e in events:
        reason = None
        if e['kind'] == 'model':
            new = (e.get('model_id'),e.get('model_hash'),e.get('session'),e.get('phase'))
            if identity is not None and new != identity: reason='model_session_or_phase_change'
            identity=new
            if e.get('sequence_gap'): reason='model_sequence_gap'
            if not e.get('time_valid',True): reason='invalid_model_event'
        elif e['kind']=='mode':
            new = (e.get('state'),e.get('phase'))
            if mode is not None and new!=mode: reason='state_change'
            mode=new
        elif e['kind']=='command' and e.get('phase','unknown') != 'unknown':
            if command_phase is not None and e['phase']!=command_phase: reason='command_phase_change'
            command_phase=e['phase']
        elif e['kind']=='barrier': reason=e['reason']
        if reason: boundaries[e['t']]=reason
    current=[]; reason='epoch_start'
    for e in events:
        if e['t'] in boundaries and (not current or e['t']>current[-1]['t']):
            if current: yield reason,current
            current=[]; reason=boundaries[e['t']]
        current.append(e)
    if current: yield reason,current

def threshold(explicit, period, factor):
    # Data-derived freshness heuristic, not a hardware guarantee or accuracy criterion.
    return explicit/1000 if explicit is not None else (factor*period if period else 0.)

def compare_block(label, error, valid, weights, span, times, reference, actual, gap, options):
    rows={}
    for name, indices in {**{n:[j] for j,n in enumerate(BODY)}, **GROUPS}.items():
        ix=np.asarray(indices); mask=valid[:,ix]
        stat=metrics(np.degrees(np.where(mask,error[:,ix],np.nan)))
        covered=float(np.sum(weights*mask.all(axis=1)))
        stat.update(valid_frames=int(mask.all(axis=1).sum()), received_frames=len(times),
                    sample_coverage=float(mask.mean()) if mask.size else 0.,
                    all_joints_time_coverage_s=covered,
                    time_coverage_fraction=covered/span if span>0 else None)
        if len(ix)==1:
            j=ix[0]
            stat['lag']=estimate(times,np.degrees(reference[:,j]),np.degrees(actual[:,j]),valid[:,j],
                                 max_lag=options.lag_max_ms/1000,step=options.lag_step_ms/1000,max_gap=gap)
        else:
            delays=[rows[BODY[j]]['lag']['estimated_lag_ms'] for j in ix
                    if rows[BODY[j]]['lag']['estimated_lag_ms'] is not None]
            stat['lag']={'status':'median_of_estimable_joints' if delays else 'insufficient',
                         'estimated_lag_ms':float(np.median(delays)) if delays else None,
                         'estimable_joints':len(delays),'total_joints':len(ix)}
        rows[name]=stat
    return rows

def analyze(database, output, options=None):
    options=options or Options(); out=Path(output)
    if out.exists() and any(out.iterdir()): raise FileExistsError('analysis output must be new or empty: '+str(out))
    for k,v in asdict(options).items():
        if k.endswith('_ms') and v is not None and (not np.isfinite(v) or v<=0): raise ValueError(k+' must be positive')
    if options.lag_step_ms>options.lag_max_ms: raise ValueError('lag step exceeds search range')
    if any(v is not None and not np.isfinite(v) for v in (options.start,options.end)):
        raise ValueError('time selection must be finite')
    if options.start is not None and options.end is not None and options.end<=options.start:
        raise ValueError('end must be after start')
    reference=None
    if options.sonic_reference is not None:
        from .sonic_reference import SonicReference
        reference=SonicReference(options.sonic_reference)
    out.mkdir(parents=True,exist_ok=True)
    db=open_db(database,readonly=True)
    source=json.loads(db.execute("SELECT value FROM metadata WHERE key='import'").fetchone()[0])
    result=dict(schema=1, angle_storage_unit='rad',metric_unit='deg',
        measurement='Actuator encoder execution following, not independent spatial accuracy',
        accuracy_acceptance='not_assessed_no_user_threshold',options=asdict(options),source=source,segments=[])
    charts=[]; arrays={}; total_reasons=Counter(); total_feedback=0; total_model_frames=0; total_command_frames=0
    total_reference_frames=0; total_reference_inputs=0; reference_reasons=Counter()
    result['test_events']=[]
    from .diagnostics import TemperatureSummary, summarize_events
    temperatures=TemperatureSummary()
    for epoch, in db.execute('SELECT DISTINCT epoch FROM events ORDER BY epoch'):
        events=list(load_epoch(db,epoch))
        for event in events:temperatures.add(event)
        result['test_events'].extend(e for e in events if e['kind']=='test_event' and
            (options.test_state is None or (e.get('state') == options.test_state and
             (options.start is None or e['t'] >= options.start))))
        if source.get("metadata.json", {}).get("model_link_method") == "mod_cycle_window":
            from .cycles import link_commands
            link_commands(events)
        links={e['source_stamp_ns']:e for e in events if e['kind']=='command_link'}
        for e in events:
            if e['kind']=='command' and e.get('source_stamp_ns') in links:
                link=links[e['source_stamp_ns']]
                for key in ('model_id','model_hash','model_session','model_seq','phase','state','association'):
                    if key in link:e[key]=link[key]
        # Thresholds come from this clock epoch before model/time selection.
        rates={k:timing([e for e in events if e['kind']==k]) for k in ('actual','command','model')}
        prior={}
        for e in events:
            if e['kind']=='model':
                key=(e.get('model_id'),e.get('model_hash'),e.get('session'))
                if e.get('sequence_gap') and key in prior:
                    before=prior[key]
                    period=rates['model']['median_period_s'] or 0.
                    before['valid_until']=before['t']+period
                prior[key]=e
        ca=threshold(options.command_age_ms,rates['command']['median_period_s'],3)
        ma=threshold(options.model_age_ms,rates['model']['median_period_s'],3)
        gap=threshold(options.feedback_gap_ms,rates['actual']['median_period_s'],5)
        for boundary, block in segments(events):
            models=[e for e in block if e['kind']=='model']
            model_id=models[0].get('model_id') if models else None
            if options.model and model_id!=options.model: continue
            states=[e for e in block if e['kind']=='actual' and
                    (options.start is None or e['t']>=options.start) and
                    (options.end is None or e['t']<=options.end)]
            if not states: continue
            reference_info=reference.attach(models) if reference else None
            commands=[e for e in block if e['kind']=='command']
            usable_models=[e for e in models if e.get('time_valid',True)]
            usable_commands=[e for e in commands if e.get('time_valid',True)]
            t=np.array([e['t'] for e in states]); n=len(t); total_feedback+=n
            q_actual=np.stack([vector(e['q'],e['names']) for e in states])
            q_command=np.full((n,29),np.nan); q_model=q_command.copy(); kp=q_command.copy()
            if reference:
                q_ref=q_command.copy(); reference_age=np.full(n,np.nan)
                reference_reason=np.full(n,'model_not_associated',dtype='<U48')
            cmd_age=np.full(n,np.nan); model_age=cmd_age.copy()
            enabled=np.zeros((n,29),bool); phase=np.full(n,'unknown',dtype='<U32')
            model_ok=np.zeros(n,bool); command_ok=np.zeros(n,bool)
            lineage=np.full(n,'none',dtype='<U24'); reasons=Counter()
            ci=causal_indices([c['t'] for c in usable_commands],t,max_age=ca)
            mt=np.array([m['t'] for m in usable_models])
            mi=causal_indices(mt,[c['t'] for c in usable_commands],max_age=ma)
            identities={(m.get('session'),m.get('model_id'),m.get('model_hash'),m.get('seq')):i for i,m in enumerate(usable_models)}
            mapped_c=[vector(c['q'],c['names']) for c in usable_commands]
            mapped_m=[vector(m['q'],m['names']) for m in usable_models]
            mapped_kp=[vector(c['kp'],c['names']) if c.get('kp') is not None else np.full(29,np.nan) for c in usable_commands]
            for i,cix in enumerate(ci):
                if cix<0:
                    reasons['missing_or_stale_command']+=1;continue
                c=usable_commands[cix];q_command[i]=mapped_c[cix];kp[i]=mapped_kp[cix]
                cmd_age[i]=t[i]-c['t'];command_ok[i]=True
                enabled[i]=np.isfinite(kp[i])&(kp[i]>0)
                if c.get('enabled') is False: enabled[i]=False
                elif c.get('enabled') is True and c.get('kp') is None: enabled[i]=True
                if c.get('kp') is None and c.get('enabled') is None: reasons['enable_state_unknown']+=1
                if not enabled[i].all(): reasons['one_or_more_joints_not_position_enabled']+=1
                phase[i]=c.get('phase','unknown')
                mix=mi[cix]
                if c.get('association') in ('same_control_cycle', 'mod_cycle_window') and c.get('model_seq') is None:
                    reasons['no_model_in_control_cycle']+=1;continue
                if c.get('model_seq') is not None:
                    mix=identities.get((c.get('model_session'),c.get('model_id'),c.get('model_hash'),c['model_seq']),-1)
                    lineage[i]='mod_cycle_window' if c.get('association') == 'mod_cycle_window' else 'explicit_sequence'
                else: lineage[i]='causal_time_only'
                if mix<0:
                    reasons['model_output_unobserved_or_stale']+=1;continue
                m=usable_models[mix]; age=c['t']-m['t']
                if c['t']>=m.get('valid_until',float('inf')):
                    reasons['model_sequence_gap_interval']+=1;continue
                if age < -1e-9 or age>ma:
                    reasons['model_age_invalid']+=1;continue
                model_age[i]=age
                if c.get('model_id') is not None and c['model_id']!=m['model_id']:
                    reasons['model_identity_mismatch']+=1;continue
                q_model[i]=mapped_m[mix]
                if reference:
                    q_ref[i]=m['_q_ref']; reference_reason[i]=m['_reference_reason']
                    reference_age[i]=t[i]-m['t']
                if phase[i]=='unknown':phase[i]=m.get('phase','unknown')
                model_ok[i]=bool(m.get('position_target',False))
                if not model_ok[i]:reasons['model_has_no_position_target']+=1
            good_time=np.array([e.get('time_valid',True) for e in states])
            hardware_enabled=np.array([e.get('enabled') is not False for e in states])
            reasons['hardware_enable_not_observed']+=sum(e.get('enabled') is None for e in states)
            reasons['hardware_explicitly_disabled']+=int((~hardware_enabled).sum())
            finite_actual=np.isfinite(q_actual)
            phase_ok=np.isin(phase,options.include_phases)
            command_phase_ok=phase_ok|(phase=='unknown')
            reasons['invalid_feedback_time']+=int((~good_time).sum())
            reasons['feedback_missing_joint_frames']+=int((~finite_actual.all(axis=1)).sum())
            for p in set(phase.tolist()): reasons['phase:'+p]+=int((phase==p).sum())
            common=finite_actual&enabled&(good_time&hardware_enabled)[:,None]
            vc=common&np.isfinite(q_command)&(command_ok&command_phase_ok)[:,None]
            vm=common&np.isfinite(q_model)&(model_ok&phase_ok)[:,None]
            vcm=vm&np.isfinite(q_command)
            weights=np.r_[np.diff(t),0.]; weights[(weights<=0)|(weights>gap)]=0.
            span=float(t[-1]-t[0]); sid=len(result['segments'])
            comparisons={}
            for label,err,mask,ref,actual in (
                ('actual_minus_model',q_actual-q_model,vm,q_model,q_actual),
                ('actual_minus_command',q_actual-q_command,vc,q_command,q_actual),
                ('command_minus_model',q_command-q_model,vcm,q_model,q_command)):
                comparisons[label]=compare_block(label,err,mask,weights,span,t,ref,actual,gap,options)
            if reference:
                reference_available=reference_reason=='available'
                vr=vm&np.isfinite(q_ref)&reference_available[:,None]
                for label,target in [('actual_minus_reference',q_actual),('model_minus_reference',q_model),
                                     ('command_minus_reference',q_command)]:
                    comparisons[label]=compare_block(label,target-q_ref,vr&np.isfinite(target),weights,span,t,q_ref,target,gap,options)
                reference_info.update(status='available' if vr.all(axis=1).any() else 'unavailable',
                    valid_body29_frames=int(vr.all(axis=1).sum()),
                    associated_reference_frames=int(reference_available.sum()),
                    association_status_counts=dict(Counter(reference_reason.tolist())))
                total_reference_frames+=reference_info['valid_body29_frames']
                total_reference_inputs+=reference_info['associated_reference_frames']
                reference_reasons.update(reference_info['association_status_counts'])
            segment=dict(id=sid,epoch=epoch,boundary=boundary,model_id=model_id,
                model_hash=models[0].get('model_hash') if models else None,
                adapter=models[0].get('adapter') if models else None,
                model_sample_stage=models[0].get('stage') if models else None,
                phase=models[0].get('phase','unknown') if models else 'unknown',
                start_s=float(t[0]),end_s=float(t[-1]),span_s=span,
                frequencies={k:timing([e for e in block if e['kind']==k]) for k in ('actual','command','model')},
                thresholds=dict(command_age_s=ca,model_age_s=ma,feedback_gap_s=gap,
                    basis='Explicit CLI values, otherwise 3× median command/model interval and 5× median feedback interval per clock epoch; heuristic freshness only'),
                enable_evidence='Explicit flags when present; otherwise positive command KP is only evidence of position-control intent. Reviewed ROS feedback has no hardware enable flag; unknown hardware enable is counted, not asserted.',
                inference_events=len(models),feedback_events=n,reasons=dict(reasons),
                independent_inference_events=len({(m.get('session'),m.get('model_id'),m.get('model_hash'),m.get('seq'))
                    for m in usable_models if m.get('position_target') and m.get('phase')=='active'}),
                lineage=dict(Counter(lineage.tolist())),comparisons=comparisons)
            if reference:segment['sonic_reference']=reference_info
            result['segments'].append(segment);total_reasons.update(reasons)
            total_model_frames+=int(vm.all(axis=1).sum());total_command_frames+=int(vc.all(axis=1).sum())
            for key,a in dict(t=t,q_model=q_model,q_command=q_command,q_actual=q_actual,
                              valid_model=vm,valid_command=vc,command_age_s=cmd_age,model_age_s=model_age,phase=phase).items():
                arrays[f's{sid}_{key}']=a
            if reference:
                for key,a in dict(q_ref=q_ref,valid_reference=vr,reference_age_s=reference_age,
                                  reference_reason=reference_reason).items():arrays[f's{sid}_{key}']=a
            ix=np.unique(np.linspace(0,n-1,min(n,1600),dtype=int))
            charts.append(dict(id=sid,model=model_id,phase=segment['phase'],t=t[ix],
                               q_model=np.degrees(q_model[ix]),q_command=np.degrees(q_command[ix]),
                               q_actual=np.degrees(q_actual[ix]),valid_model=vm[ix],valid_command=vc[ix]))
            if reference:charts[-1].update(q_ref=np.degrees(q_ref[ix]),valid_reference=vr[ix])
    db.close()
    result['summary']=dict(feedback_frames=total_feedback,model_valid_body29_frames=total_model_frames,
        command_valid_body29_frames=total_command_frames,reasons=dict(total_reasons),
        model_result='available' if total_model_frames else '模型输出未观测或无有效29维角度目标',
        command_result='available' if total_command_frames else '无有效29维指令跟踪样本')
    if reference:
        status=('available' if total_reference_frames==total_feedback else 'partial') if total_reference_frames else 'unavailable'
        result['sonic_reference']=dict(reference.metadata,status=status,
            inference_events=sum(reference.counts.values()),inference_status_counts=dict(reference.counts),
            valid_body29_frames=total_reference_frames,associated_reference_frames=total_reference_inputs,
            association_status_counts=dict(reference_reasons),decoder_errors=reference.errors)
        result['summary'].update(reference_result=status,reference_valid_body29_frames=total_reference_frames)
        reference.save(out/'sonic_reference.npz')
    capture=source.get('capture_result.json',{})
    result['temperatures']=temperatures.report(source)
    result['diagnostics']=summarize_events(result['test_events'])
    issues={k:v for k,v in source['counts'].items() if v and
            any(k.startswith(p) for p in ('rejected:','invalid_','out_of_order','duplicate','model_sequence_missing','ros_clock','receive_clock','missing_joint'))}
    result['data_integrity']=dict(import_issues=issues,recorder=capture,
        complete_body29_model_coverage=bool(total_feedback and total_model_frames==total_feedback),
        note='Coverage and data integrity are separate from control accuracy; missing samples are never zero errors. ROS source time is mapped to host monotonic time, not motor execution time.')
    if options.acceptance_profile is not None:
        from .acceptance import PROFILE, assess
        profile=(PROFILE if options.acceptance_profile==PROFILE['id']
                 else json.loads(Path(options.acceptance_profile).read_text()))
        result['acceptance']=assess(result,profile)
        result['accuracy_acceptance']=result['acceptance']['status']
        result['summary']['accuracy_screening']=result['acceptance']['status']
        result['diagnostics']['precision_exceedances']=[
            dict(code='TRACKING_SCREENING_EXCEEDED',category='tracking',severity='error',
                 origin='offline_analysis',automatic_control_action='none',
                 segment=segment['segment'],model_id=segment['model_id'],joint=name,
                 comparison=profile['comparison'],violations=joint['violations'])
            for segment in result['acceptance']['segments'] if segment['status']=='screening_fail'
            for name,joint in segment['joints'].items() if joint['violations']]
    dump(out/'report.json',result)
    np.savez_compressed(out/'aligned.npz',**arrays)
    from .report import render
    render(out,result,charts)
    return result
