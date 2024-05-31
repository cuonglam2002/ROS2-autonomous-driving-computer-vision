from .a_colour_segmentation import segment_lanes
from ...config import config
import cv2
from .b_midlane_estimation import estimate_midlane
from .c_cleaning import GetYellowInnerEdge,ExtendShortLane
from .d_data_extraction import FetchInfoAndDisplay
# from .trafficsign import det_traffic_sign
def detect_lanes(img):
    # out1 = det_traffic_sign(img)
    # cv2.imshow("1",out1)
    img_cropped = img[config.CropHeight_resized:,:]
    # cv2.imshow("1", img_cropped)
    # segment_lanes(img_cropped, config.minArea_resized)
    mid_lane_mask,mid_lane_edge,outer_lane_edge,outerlane_side_sep,outerlane_points = segment_lanes(img_cropped,config.minArea_resized)
    estimated_midlane = estimate_midlane(mid_lane_edge,config.MaxDist_resized)


    # [Lane Detection] STAGE_3 (Cleaning) <<<<<<--->>>>>> [STEP_1]:
    OuterLane_OneSide,Outer_cnts_oneSide,Mid_cnts,Offset_correction = GetYellowInnerEdge(outerlane_side_sep,estimated_midlane,outerlane_points)#3ms
    # [Lane Detection] STAGE_3 (Cleaning) <<<<<<--->>>>>> [STEP_2]:
    extended_midlane,extended_outerlane = ExtendShortLane(estimated_midlane,Mid_cnts,Outer_cnts_oneSide,OuterLane_OneSide.copy())

    Distance , Curvature = FetchInfoAndDisplay(mid_lane_edge,extended_midlane,extended_outerlane,img_cropped,Offset_correction)
    # det_traffic_sign(img)
    
    # cv2.imshow("mid_lane_mask",mid_lane_mask)
    # cv2.imshow("mid_lane_edge",mid_lane_edge)
    # cv2.imshow("outer_lane_edge",outer_lane_edge)
    # cv2.imshow("outerlane_side_sep",outerlane_side_sep)
    # cv2.imshow("estimated_midlane",estimated_midlane)

    # cv2.imshow("OuterLane_OneSide",OuterLane_OneSide)
    # cv2.imshow("extended_midlane",extended_midlane)
    # cv2.imshow("extended_outerlane",extended_outerlane)

    # cv2.waitKey(5)
    return Distance,Curvature