from array import array

import numpy as np
import pytest

from bxi_example_py_elf3.framework.joints import JointLayout, JointTargetBuffer
from bxi_example_py_elf3.framework.platform.joint_io import (
    FixedOrderJointCommandEncoder,
    FixedOrderJointStateSource,
    NamedJointStateSource,
)


@pytest.mark.parametrize("source_kind", ["fixed", "named"])
def test_rejected_sample_keeps_previous_observation(source_kind):
    layout = JointLayout.create(["a", "b"])
    if source_kind == "fixed":
        source = FixedOrderJointStateSource(layout)
        update = source.update
    else:
        source = NamedJointStateSource(layout)
        def update(position, velocity, **kwargs):
            return source.update(layout.names, position, velocity, **kwargs)

    view = update([1, 2], [3, 4], timestamp_ns=5)
    for velocity, stamp in [([9], 6), (["bad", "data"], 6), ([7, 8], None)]:
        with pytest.raises((ValueError, TypeError)):
            update([8, 9], velocity, timestamp_ns=stamp)
        assert view.position.tolist() == [1, 2]
        assert view.velocity.tolist() == [3, 4]
        assert view.timestamp_ns == 5


def test_first_valid_frame_establishes_layout_and_reordering():
    source = NamedJointStateSource()
    with pytest.raises(ValueError):
        source.update(["invalid"], [], [])
    assert not source.ready
    view = source.update(["a", "b"], array("d", [1, 2]), array("d", [3, 4]))
    with pytest.raises(ValueError):
        source.update(["b", "a"], [20, 10], [])
    assert view.position.tolist() == [1, 2]
    assert view is source.update(["b", "a"], [20, 10], [40, 30], timestamp_ns=8)
    assert view.position.tolist() == [10, 20]
    assert view.velocity.tolist() == [30, 40]
    assert view.timestamp_ns == 8


def test_fixed_encoder_reorders_all_fields_in_a_stable_view():
    layout = JointLayout.create(["a", "b"])
    encoder = FixedOrderJointCommandEncoder(layout)
    target = JointTargetBuffer(JointLayout.create(["b", "a"]))
    view = encoder.encode(target.update([1, 2], [3, 4], [5, 6]))
    assert view is encoder.target
    np.testing.assert_array_equal(view.position, [2, 1])
    np.testing.assert_array_equal(view.kp, [4, 3])
    np.testing.assert_array_equal(view.kd, [6, 5])
    with pytest.raises(ValueError):
        encoder.encode(JointTargetBuffer(JointLayout.create(["a"])).view)
