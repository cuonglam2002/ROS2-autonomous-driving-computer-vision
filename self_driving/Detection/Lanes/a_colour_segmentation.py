import cv2
import numpy as np

from .Morph_op import BwareaOpen, Ret_LowestEdgePoints,RetLargestContour_OuterLane


hls = 0
src = 0

#White Regions Range 
lit_h = 255
hue_l = 50
lit_l = 140
sat_l = 10#

hue_l_y = 20
hue_h_y = 40
lit_l_y = 80
sat_l_y = 20


def clr_segment(hls,lower_range,upper_range):
    mask_in_range = cv2.inRange(hls,lower_range,upper_range)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    mask_dilated = cv2.morphologyEx(mask_in_range,cv2.MORPH_DILATE,kernel)
    return mask_dilated


def maskextract():
    mask   = clr_segment(hls,(hue_l  ,lit_l   ,sat_l  ),(255       ,lit_h,255))
    mask_y = clr_segment(hls,(hue_l_y,lit_l_y ,sat_l_y),(hue_h_y,255,255))#combine 6ms

    mask_ = mask != 0
    dst = src * (mask_[:,:,None].astype(src.dtype))

    mask_y_ = mask_y != 0
    dst_Y = src * (mask_y_[:,:,None].astype(src.dtype))

    
def on_hue_low_change(val):
    global hue_l
    hue_l = val
    maskextract()
def on_lit_high_change(val):
    global lit_h
    lit_h = val
    maskextract()
    
def on_lit_low_change(val):
    global lit_l
    lit_l = val
    maskextract()
def on_sat_low_change(val):
    global sat_l
    sat_l = val
    maskextract()
    
def on_hue_low_y_change(val):
    global hue_l_y
    hue_l_y = val
    maskextract()
def on_hue_high_y_change(val):
    global hue_h_y
    hue_h_y = val
    maskextract()
def on_lit_low_y_change(val):
    global lit_l_y
    lit_l_y = val
    maskextract()
def on_sat_low_y_change(val):
    global sat_l_y
    sat_l_y = val
    maskextract()

cv2.namedWindow("white_regions")
cv2.namedWindow("yellow_regions")

cv2.createTrackbar("Hue_L","white_regions",hue_l,255,on_hue_low_change)
cv2.createTrackbar("Lit_L","white_regions",lit_l,255,on_lit_low_change)
cv2.createTrackbar("Lit_H","white_regions",lit_h,255,on_lit_high_change)
cv2.createTrackbar("Sat_L","white_regions",sat_l,255,on_sat_low_change)

cv2.createTrackbar("Hue_L_Y","yellow_regions",hue_l_y,255,on_hue_low_y_change)
cv2.createTrackbar("Hue_H_Y","yellow_regions",hue_h_y,255,on_hue_high_y_change)
cv2.createTrackbar("Lit_L_Y","yellow_regions",lit_l_y,255,on_lit_low_y_change)
cv2.createTrackbar("Sat_L_Y","yellow_regions",sat_l_y,255,on_sat_low_y_change)

def get_mask_nd_edge_of_largerobjects_w(frame,mask,min_area):
    # Keeping only objects larger then min_area
    frame_roi = cv2.bitwise_and(frame,frame,mask=mask)
    _, frame_roi = cv2.threshold(frame_roi, 0, 255, cv2.THRESH_BINARY)
    frame_roi_gray = cv2.cvtColor(frame_roi,cv2.COLOR_BGR2GRAY)
    mask_of_larger_objects = BwareaOpen(frame_roi_gray, min_area)
    frame_roi_gray = cv2.bitwise_and(frame_roi_gray, mask_of_larger_objects)
    
    # Extracting Edges of those larger objects
    frame_roi_smoothed = cv2.GaussianBlur(frame_roi_gray,(11,11),1)
    edges_of_larger_objects = cv2.Canny(frame_roi_smoothed,50,150, None, 3)

    return mask_of_larger_objects,edges_of_larger_objects
def get_mask_nd_edge_of_largerobjects_y(frame,mask,min_area):
    # Keeping only objects larger then min_area
    frame_roi = cv2.bitwise_and(frame,frame,mask=mask)
    frame_roi_gray = cv2.cvtColor(frame_roi,cv2.COLOR_BGR2GRAY)
    mask_of_larger_objects = BwareaOpen(frame_roi_gray, min_area)
    frame_roi_gray = cv2.bitwise_and(frame_roi_gray, mask_of_larger_objects)
    
    # Extracting Edges of those larger objects
    frame_roi_smoothed = cv2.GaussianBlur(frame_roi_gray,(11,11),1)
    edges_of_larger_objects = cv2.Canny(frame_roi_smoothed,50,150, None, 3)

    return mask_of_larger_objects,edges_of_larger_objects

def segment_midlane(frame,white_regions,min_area):
    mid_lane_mask ,mid_lane_edge = get_mask_nd_edge_of_largerobjects_w(frame,white_regions,min_area)
    return mid_lane_mask,mid_lane_edge

def segment_outerlane(frame,yellow_regions,min_area):
    outer_points_list = []
    mask,edges = get_mask_nd_edge_of_largerobjects_y(frame,yellow_regions,min_area)
    mask_largest, largest_found = RetLargestContour_OuterLane(mask,min_area)

    if largest_found:
        # Keep only edges of largest region
        edge_largest= cv2.bitwise_and(edges,mask_largest)
        # Return edge points for identifying closest edge later
        lanes_sides_sep,outer_points_list= Ret_LowestEdgePoints(mask_largest)
        edges = edge_largest
    else:
        lanes_sides_sep = np.zeros((frame.shape[0],frame.shape[1]),np.uint8)
    
    return edges,lanes_sides_sep,outer_points_list




def segment_lanes(frame,min_area):
    global hls,src
    src = frame.copy()

    hls = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)

    # Segmenting White regions
    white_regions = clr_segment(hls,np.array([hue_l,lit_l,sat_l]),np.array([255,lit_h,255]))
    yellow_regions = clr_segment(hls,np.array([hue_l_y,lit_l_y,sat_l_y]),np.array([hue_h_y,255,255]))

    cv2.imshow("white_regions",white_regions)
    cv2.imshow("yellow_regions",yellow_regions)
    
    

    # # Semgneting midlane from white regions
    mid_lane_mask,mid_lane_edge = segment_midlane(frame,white_regions,min_area)


    # # Semgneting outerlane from yellow regions
    outer_lane_edge,outerlane_side_sep,outerlane_points= segment_outerlane(frame,yellow_regions,min_area+500)  
    
   

    return mid_lane_mask,mid_lane_edge,outer_lane_edge,outerlane_side_sep,outerlane_points
    
