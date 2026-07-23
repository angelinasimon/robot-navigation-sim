from robot_nav_sim.eval_utils import iou

def test_iou_known_overlap():
    box_a = (0, 0, 2, 2)
    box_b = (1, 1, 3, 3)

    assert abs(iou(box_a, box_b) - (1 / 7)) < 1e-6