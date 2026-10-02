#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "remote_controller/input_mapper.hpp"

namespace {

using remote_controller::ConditionConfig;
using remote_controller::ControlConfig;
using remote_controller::ControlInputConfig;
using remote_controller::ControlInputKind;
using remote_controller::EnumPositionConfig;
using remote_controller::InputMapper;
using remote_controller::RemoteConfig;

ControlInputConfig source_input(const std::string &source, int priority = 0)
{
    ControlInputConfig input;
    input.name = source;
    input.priority = priority;
    input.kind = ControlInputKind::kSource;
    input.source.source = source;
    return input;
}

ControlInputConfig conditional_input(
    const ConditionConfig &condition,
    double value,
    int priority = 0)
{
    ControlInputConfig input;
    input.name = "conditional";
    input.priority = priority;
    input.kind = ControlInputKind::kConditionalValue;
    input.when = condition;
    input.analog_value = value;
    return input;
}

ConditionConfig equals(const std::string &control, const std::string &value)
{
    ConditionConfig condition;
    condition.kind = "equals";
    condition.control = control;
    condition.value = value;
    return condition;
}

ConditionConfig raw_range(const std::string &source, double min, double max)
{
    ConditionConfig condition;
    condition.kind = "raw_range";
    condition.source = source;
    condition.min = min;
    condition.max = max;
    return condition;
}

EnumPositionConfig position(const std::string &value, double min, double max)
{
    EnumPositionConfig result;
    result.value = value;
    result.min = min;
    result.max = max;
    return result;
}

double yaw(InputMapper &mapper)
{
    communication::msg::MotionCommands message;
    mapper.fill_message(message);
    return message.yawdot_des;
}

void expect_close(double actual, double expected)
{
    if (std::fabs(actual - expected) > 1e-6) {
        std::abort();
    }
}

void expect(bool value)
{
    if (!value) {
        std::abort();
    }
}

void test_enum_conditions_are_independent_of_raw_gaps()
{
    RemoteConfig config;

    ControlConfig group;
    group.name = "button_group";
    group.type = "enum";
    group.mix = "first_active";
    group.default_enum = "idle";
    group.inputs.push_back(source_input("raw.group"));
    group.positions = {
        position("idle", -0.1, 0.1),
        position("dpad_up", 0.2, 0.3),
        position("dpad_down", 0.4, 0.5),
        position("dpad_left", 0.6, 0.7),
        position("dpad_right", 0.9, 1.0),
    };
    config.controls.push_back(group);

    ControlConfig yaw_control;
    yaw_control.name = "yaw";
    yaw_control.type = "analog";
    yaw_control.mix = "sum";
    yaw_control.inputs.push_back(conditional_input(equals("button_group", "dpad_left"), 1.0));
    yaw_control.inputs.push_back(conditional_input(equals("button_group", "dpad_right"), -1.0));
    config.controls.push_back(yaw_control);

    remote_controller::AnalogOutputConfig output;
    output.field = "yawdot_des";
    output.controls = {"yaw"};
    config.analog_outputs.push_back(output);

    InputMapper mapper(config);
    mapper.set_signal("raw.group", 0.65);
    expect_close(yaw(mapper), 1.0);

    mapper.set_signal("raw.group", 0.95);
    expect_close(yaw(mapper), -1.0);

    mapper.set_signal("raw.group", 0.25);
    expect_close(yaw(mapper), 0.0);
}

void test_priority_preempts_lower_active_inputs()
{
    RemoteConfig config;
    ControlConfig control;
    control.name = "priority_axis";
    control.type = "analog";
    control.mix = "sum";
    control.inputs.push_back(source_input("raw.axis", 0));
    control.inputs.push_back(conditional_input(raw_range("raw.override", 0.5, 1.0), -1.0, 100));
    config.controls.push_back(control);

    remote_controller::AnalogOutputConfig output;
    output.field = "yawdot_des";
    output.controls = {"priority_axis"};
    config.analog_outputs.push_back(output);

    InputMapper mapper(config);
    mapper.set_signals({{"raw.axis", 0.8}, {"raw.override", 0.0}});
    expect_close(yaw(mapper), 0.8);

    mapper.set_signal("raw.override", 0.7);
    expect_close(yaw(mapper), -1.0);
}

void test_bool_all_keeps_inactive_raw_inputs_in_the_selected_group()
{
    RemoteConfig config;

    ControlConfig both;
    both.name = "both";
    both.type = "bool";
    both.mix = "all";
    both.inputs.push_back(source_input("raw.left"));
    both.inputs.push_back(source_input("raw.right"));
    config.controls.push_back(both);

    ControlConfig gate;
    gate.name = "gate";
    gate.type = "analog";
    gate.inputs.push_back(conditional_input(
        []() {
            ConditionConfig condition;
            condition.kind = "pressed";
            condition.control = "both";
            return condition;
        }(),
        1.0));
    config.controls.push_back(gate);

    remote_controller::AnalogOutputConfig output;
    output.field = "yawdot_des";
    output.controls = {"gate"};
    config.analog_outputs.push_back(output);

    InputMapper mapper(config);
    mapper.set_signals({{"raw.left", 1.0}, {"raw.right", 0.0}});
    expect_close(yaw(mapper), 0.0);

    mapper.set_signal("raw.right", 1.0);
    expect_close(yaw(mapper), 1.0);
}

void test_debug_reports_changed_rule_selection()
{
    RemoteConfig config;
    ControlConfig control;
    control.name = "debug_axis";
    control.type = "analog";
    control.inputs.push_back(conditional_input(raw_range("raw.axis", 0.1, 1.0), 0.5));
    config.controls.push_back(control);

    InputMapper mapper(config);
    mapper.set_debug_enabled(true);
    mapper.take_debug_messages();
    mapper.set_signal("raw.axis", 0.5);
    const std::vector<std::string> messages = mapper.take_debug_messages();
    expect(!messages.empty());
    expect(messages.front().find("debug_axis") != std::string::npos);
}

void test_suspended_face_buttons_keep_existing_outputs()
{
    const RemoteConfig config = remote_controller::load_remote_config(
        REMOTE_CONTROLLER_TEST_CONFIG_PATH);

    {
        InputMapper mapper(config);
        mapper.set_signal("js.button.0", 1.0);
        communication::msg::MotionCommands message;
        mapper.fill_message(message);
        expect(message.btn_7 == 1);
        expect(message.btn_9 == 0);
        expect(message.btn_10 == 0);
    }

    {
        InputMapper mapper(config);
        mapper.set_signal("js.button.3", 1.0);
        communication::msg::MotionCommands message;
        mapper.fill_message(message);
        expect(message.btn_7 == 0);
        expect(message.btn_9 == 1);
        expect(message.btn_10 == 0);
    }

    {
        InputMapper mapper(config);
        mapper.set_signal("js.button.4", 1.0);
        communication::msg::MotionCommands message;
        mapper.fill_message(message);
        expect(message.btn_7 == 0);
        expect(message.btn_9 == 0);
        expect(message.btn_10 == 1);
    }
}

void test_b_button_selects_sequence_without_shoulders()
{
    const RemoteConfig config = remote_controller::load_remote_config(
        REMOTE_CONTROLLER_TEST_CONFIG_PATH);
    InputMapper mapper(config);
    mapper.set_signal("js.button.1", 1.0);
    communication::msg::MotionCommands message;
    mapper.fill_message(message);
    expect(message.btn_1 == 0);
    expect(message.btn_2 == 0);
    expect(message.btn_3 == 0);
    expect(message.btn_4 == 0);
    expect(message.btn_5 == 0);
    expect(message.btn_6 == 0);
    expect(message.btn_7 == 0);
    expect(message.btn_8 == 3);
    expect(message.btn_9 == 2);
    expect(message.btn_10 == 0);
}

void test_basic_and_test_mode_buttons_do_not_overlap()
{
    const RemoteConfig config = remote_controller::load_remote_config(
        REMOTE_CONTROLLER_TEST_CONFIG_PATH);

    const auto buttons_for = [&config](
        const std::vector<std::pair<std::string, double>> &signals) {
        InputMapper mapper(config);
        mapper.set_signals(signals);
        communication::msg::MotionCommands message;
        mapper.fill_message(message);
        return std::vector<int>{
            message.btn_1, message.btn_2, message.btn_3, message.btn_4,
            message.btn_5, message.btn_6, message.btn_7, message.btn_8,
            message.btn_9, message.btn_10,
        };
    };

    const auto only = [](const std::vector<int> &buttons, int index, int release_marker = 0) {
        for (int i = 0; i < 10; ++i) {
            expect(buttons[i] == (i == index - 1 ? 1 : (i == 7 ? release_marker : 0)));
        }
    };

    only(buttons_for({{"js.button.7", 1.0}, {"js.button.3", 1.0}}), 1, 3);
    only(buttons_for({{"js.button.7", 1.0}, {"js.button.0", 1.0}}), 2, 3);
    only(buttons_for({{"js.button.7", 1.0}, {"js.button.1", 1.0}}), 3, 3);
    only(buttons_for({{"js.button.7", 1.0}, {"js.button.4", 1.0}}), 4, 3);
    only(buttons_for({{"js.button.6", 1.0}, {"js.button.0", 1.0}}), 6, 3);
    only(buttons_for({{"js.button.6", 1.0}, {"js.button.7", 1.0},
                      {"js.button.4", 1.0}}), 8);
    const auto exit_buttons = buttons_for({{"js.button.6", 1.0}, {"js.button.7", 1.0},
                                           {"js.button.1", 1.0}});
    for (int i = 0; i < 10; ++i) {
        expect(exit_buttons[i] == (i == 7 ? 2 : 0));
    }
    const auto forward_buttons = buttons_for({{"js.button.6", 1.0}, {"js.button.7", 1.0},
                                               {"js.button.3", 1.0}});
    expect(forward_buttons[4] == 1);
    expect(forward_buttons[7] == 3);
    only(buttons_for({{"js.button.0", 1.0}}), 7);
    only(buttons_for({{"js.button.3", 1.0}}), 9);
    only(buttons_for({{"js.button.4", 1.0}}), 10);
    const auto sequence_buttons = buttons_for({{"js.button.1", 1.0}});
    expect(sequence_buttons[7] == 3 && sequence_buttons[8] == 2);

    expect(buttons_for({{"js.button.6", 1.0}})[7] == 3);
    expect(buttons_for({{"js.button.7", 1.0}})[7] == 3);
    expect(buttons_for({{"js.button.1", 1.0}})[7] == 3);
    expect(buttons_for({})[7] == 0);

    InputMapper mapper(config);
    communication::msg::MotionCommands message;
    mapper.set_signals({{"js.button.6", 1.0}, {"js.button.7", 1.0},
                        {"js.button.1", 1.0}});
    mapper.fill_message(message);
    expect(message.btn_8 == 2);
    mapper.set_signal("js.button.6", 0.0);
    mapper.fill_message(message);
    expect(message.btn_8 == 3 && message.btn_3 == 1);
    mapper.set_signal("js.button.7", 0.0);
    mapper.fill_message(message);
    expect(message.btn_8 == 3 && message.btn_3 == 0);
    mapper.set_signal("js.button.1", 0.0);
    mapper.fill_message(message);
    expect(message.btn_8 == 0);

    mapper.set_signals({{"js.button.6", 1.0}, {"js.button.7", 1.0},
                        {"js.button.1", 1.0}});
    mapper.fill_message(message);
    expect(message.btn_8 == 2);
    mapper.set_signal("js.button.1", 0.0);
    mapper.fill_message(message);
    expect(message.btn_8 == 3);
    mapper.set_signal("js.button.6", 0.0);
    mapper.fill_message(message);
    expect(message.btn_8 == 3);
    mapper.set_signal("js.button.7", 0.0);
    mapper.fill_message(message);
    expect(message.btn_8 == 0);
}

void test_lie_down_keeps_original_rt_y_shortcut()
{
    const RemoteConfig config = remote_controller::load_remote_config(
        REMOTE_CONTROLLER_TEST_CONFIG_PATH);
    InputMapper mapper(config);
    mapper.set_signals({{"js.axis.4", 1.0}, {"js.button.4", 1.0}});
    communication::msg::MotionCommands message;
    mapper.fill_message(message);
    expect(message.btn_10 == 8);
    expect(message.btn_4 == 0);

    mapper.set_signal("js.button.7", 1.0);
    mapper.fill_message(message);
    expect(message.btn_10 != 8);
}

void test_start_buttons_select_distinct_imu_sources()
{
    const RemoteConfig config = remote_controller::load_remote_config(
        REMOTE_CONTROLLER_TEST_CONFIG_PATH);
    InputMapper mapper(config);
    expect(mapper.set_signal("js.button.14", 1.0) ==
        std::vector<std::string>{"system.start"});
    expect(mapper.set_signal("js.button.14", 0.0).empty());
    expect(mapper.set_signal("js.button.13", 1.0) ==
        std::vector<std::string>{"system.start_hardware"});

    const auto original = config.system_commands.find("start");
    const auto hardware = config.system_commands.find("start_hardware");
    expect(original != config.system_commands.end());
    expect(hardware != config.system_commands.end());
    expect(std::any_of(original->second.begin(), original->second.end(),
        [](const std::string &command) {
            return command.find("example_demo_hw.launch.py enable_imu:=false") !=
                std::string::npos;
        }));
    expect(std::any_of(original->second.begin(), original->second.end(),
        [](const std::string &command) {
            return command.find("start_imu_if_owned.sh") != std::string::npos;
        }));
    expect(std::any_of(hardware->second.begin(), hardware->second.end(),
        [](const std::string &command) {
            return command.find("example_demo_hw.launch.py enable_imu:=true") !=
                std::string::npos;
        }));
    expect(std::none_of(hardware->second.begin(), hardware->second.end(),
        [](const std::string &command) {
            return command.find("start_imu_if_owned.sh") != std::string::npos;
        }));
    expect(config.system_mutexes.size() == 1);
    expect(config.system_mutexes.front().acquire ==
        (std::vector<std::string>{"start", "start_hardware"}));
    expect(config.system_mutexes.front().release == "stop");
}

}  // namespace

int main()
{
    test_enum_conditions_are_independent_of_raw_gaps();
    test_priority_preempts_lower_active_inputs();
    test_bool_all_keeps_inactive_raw_inputs_in_the_selected_group();
    test_debug_reports_changed_rule_selection();
    test_suspended_face_buttons_keep_existing_outputs();
    test_b_button_selects_sequence_without_shoulders();
    test_basic_and_test_mode_buttons_do_not_overlap();
    test_lie_down_keeps_original_rt_y_shortcut();
    test_start_buttons_select_distinct_imu_sources();
    return 0;
}
