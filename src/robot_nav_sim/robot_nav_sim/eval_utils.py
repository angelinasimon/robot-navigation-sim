def iou(box_a, box_b):
    """
    Compute Intersection over Union (IoU) for two bounding boxes.

    Boxes are in the format:
    (x_min, y_min, x_max, y_max)
    """

    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b

    # Coordinates of the overlapping rectangle
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)

    # Width and height of the overlap (0 if no overlap)
    inter_width = max(0, inter_x2 - inter_x1)
    inter_height = max(0, inter_y2 - inter_y1)

    # Areas
    intersection = inter_width * inter_height
    area_a = (ax2 - ax1) * (ay2 - ay1)
    area_b = (bx2 - bx1) * (by2 - by1)

    union = area_a + area_b - intersection

    # Prevent division by zero for malformed boxes
    if union == 0:
        return 0.0

    return intersection / union