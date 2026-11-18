import csv
import json
from pathlib import Path
from .contract import BODY, clean

def render(out, result, charts):
    with (out/'metrics.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['segment','model','phase','comparison','joint_or_part','n','mae_deg','rmse_deg','p95_abs_deg','max_abs_deg','bias_deg','sample_coverage','time_coverage_s','lag_ms','lag_status'])
        for s in result['segments']:
            for label, rows in s['comparisons'].items():
                for name,m in rows.items():
                    writer.writerow([s['id'],s['model_id'],s['phase'],label,name,m['n'],m['mae_deg'],m['rmse_deg'],m['p95_abs_deg'],m['max_abs_deg'],m['bias_deg'],m['sample_coverage'],m['all_joints_time_coverage_s'],m['lag']['estimated_lag_ms'],m['lag']['status']])
    data=json.dumps(clean(dict(report=result,charts=charts,joints=BODY)),ensure_ascii=False,separators=(',',':'),allow_nan=False).replace('<','\\u003c')
    template=Path(__file__).with_name('report_template.html').read_text()
    (out/'report.html').write_text(template.replace('__DATA__',data))
