#!/usr/bin/env python3
"""
YOLO检测后处理器 - 包含解析、NMS和IOU跟踪
"""
import numpy as np
import cv2
from typing import List, Tuple

class Detection:
    def __init__(self, x1, y1, x2, y2, confidence, class_id):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.confidence = confidence
        self.class_id = class_id
        self.track_id = -1
    
    @property
    def width(self):
        return self.x2 - self.x1
    
    @property
    def height(self):
        return self.y2 - self.y1
    
    @property
    def center(self):
        return ((self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2)

class YOLOPostProcessor:
    def __init__(self, conf_threshold=0.5, iou_threshold=0.45):
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
    
    def parse_yolo_output(self, output, img_width, img_height, input_size=320):
        """
        解析YOLO输出
        output shape: (1, num_detections, 6) 或 (1, num_detections, 5+num_classes)
        """
        detections = []
        
        if output is None or len(output) == 0:
            return detections
        
        output = output[0]
        
        grid_size = input_size
        
        for i in range(output.shape[0]):
            detection = output[i]
            
            if len(detection) >= 6:
                cx, cy, w, h = detection[0:4]
                obj_conf = detection[4]
                class_conf = detection[5]
                class_id = 0
            elif len(detection) >= 5:
                cx, cy, w, h = detection[0:4]
                obj_conf = detection[4]
                class_conf = obj_conf
                class_id = 0
            else:
                continue
            
            confidence = obj_conf * class_conf
            
            if confidence < self.conf_threshold:
                continue
            
            x1 = int((cx - w / 2) * img_width / grid_size)
            y1 = int((cy - h / 2) * img_height / grid_size)
            x2 = int((cx + w / 2) * img_width / grid_size)
            y2 = int((cy + h / 2) * img_height / grid_size)
            
            x1 = max(0, min(x1, img_width - 1))
            y1 = max(0, min(y1, img_height - 1))
            x2 = max(0, min(x2, img_width - 1))
            y2 = max(0, min(y2, img_height - 1))
            
            if x2 > x1 and y2 > y1:
                detections.append(Detection(x1, y1, x2, y2, confidence, class_id))
        
        return detections
    
    def compute_iou(self, det1: Detection, det2: Detection) -> float:
        """计算两个检测框的IOU"""
        x1_inter = max(det1.x1, det2.x1)
        y1_inter = max(det1.y1, det2.y1)
        x2_inter = min(det1.x2, det2.x2)
        y2_inter = min(det1.y2, det2.y2)
        
        if x2_inter <= x1_inter or y2_inter <= y1_inter:
            return 0.0
        
        inter_area = (x2_inter - x1_inter) * (y2_inter - y1_inter)
        
        area1 = (det1.x2 - det1.x1) * (det1.y2 - det1.y1)
        area2 = (det2.x2 - det2.x1) * (det2.y2 - det2.y1)
        
        iou = inter_area / (area1 + area2 - inter_area + 1e-6)
        return iou
    
    def nms(self, detections: List[Detection]) -> List[Detection]:
        """非极大值抑制"""
        if len(detections) == 0:
            return []
        
        detections.sort(key=lambda x: x.confidence, reverse=True)
        
        keep = []
        suppressed = [False] * len(detections)
        
        for i in range(len(detections)):
            if suppressed[i]:
                continue
            
            keep.append(detections[i])
            
            for j in range(i + 1, len(detections)):
                if suppressed[j]:
                    continue
                
                if detections[i].class_id == detections[j].class_id:
                    iou = self.compute_iou(detections[i], detections[j])
                    if iou > self.iou_threshold:
                        suppressed[j] = True
        
        return keep

class SimpleTracker:
    def __init__(self, iou_threshold=0.3, max_age=5, min_hits=2):
        self.iou_threshold = iou_threshold
        self.max_age = max_age
        self.min_hits = min_hits
        
        self.tracks = {}
        self.next_id = 0
        self.frame_count = 0
    
    def update(self, detections: List[Detection]) -> List[Detection]:
        """更新跟踪器"""
        self.frame_count += 1
        
        if len(self.tracks) == 0:
            for det in detections:
                self._create_track(det)
            return detections
        
        matched_pairs = []
        unmatched_dets = list(range(len(detections)))
        unmatched_tracks = list(self.tracks.keys())
        
        iou_matrix = np.zeros((len(detections), len(self.tracks)))
        track_ids = list(self.tracks.keys())
        
        for i, det in enumerate(detections):
            for j, track_id in enumerate(track_ids):
                track = self.tracks[track_id]
                iou_matrix[i, j] = self._compute_iou_detection(det, track)
        
        while len(unmatched_dets) > 0 and len(unmatched_tracks) > 0:
            max_iou = 0.0
            max_i = -1
            max_j = -1
            
            for i in unmatched_dets:
                for j_idx, track_id in enumerate(track_ids):
                    if track_id in unmatched_tracks:
                        j = track_ids.index(track_id)
                        if iou_matrix[i, j] > max_iou:
                            max_iou = iou_matrix[i, j]
                            max_i = i
                            max_j = track_id
            
            if max_iou > self.iou_threshold:
                matched_pairs.append((max_i, max_j))
                unmatched_dets.remove(max_i)
                unmatched_tracks.remove(max_j)
            else:
                break
        
        for det_idx, track_id in matched_pairs:
            det = detections[det_idx]
            det.track_id = track_id
            self.tracks[track_id]['det'] = det
            self.tracks[track_id]['age'] = 0
            self.tracks[track_id]['hits'] += 1
        
        for det_idx in unmatched_dets:
            self._create_track(detections[det_idx])
        
        for track_id in unmatched_tracks:
            self.tracks[track_id]['age'] += 1
        
        tracks_to_remove = []
        for track_id, track in self.tracks.items():
            if track['age'] > self.max_age:
                tracks_to_remove.append(track_id)
        
        for track_id in tracks_to_remove:
            del self.tracks[track_id]
        
        confirmed_detections = []
        for det in detections:
            if det.track_id >= 0:
                track = self.tracks[det.track_id]
                if track['hits'] >= self.min_hits:
                    confirmed_detections.append(det)
        
        return confirmed_detections
    
    def _create_track(self, det: Detection):
        """创建新轨迹"""
        det.track_id = self.next_id
        self.tracks[self.next_id] = {
            'det': det,
            'age': 0,
            'hits': 1
        }
        self.next_id += 1
    
    def _compute_iou_detection(self, det1: Detection, track) -> float:
        """计算检测框和轨迹的IOU"""
        det2 = track['det']
        return self._compute_iou(det1, det2)
    
    def _compute_iou(self, det1: Detection, det2: Detection) -> float:
        """计算两个检测框的IOU"""
        x1_inter = max(det1.x1, det2.x1)
        y1_inter = max(det1.y1, det2.y1)
        x2_inter = min(det1.x2, det2.x2)
        y2_inter = min(det1.y2, det2.y2)
        
        if x2_inter <= x1_inter or y2_inter <= y1_inter:
            return 0.0
        
        inter_area = (x2_inter - x1_inter) * (y2_inter - y1_inter)
        
        area1 = (det1.x2 - det1.x1) * (det1.y2 - det1.y1)
        area2 = (det2.x2 - det2.x1) * (det2.y2 - det2.y1)
        
        iou = inter_area / (area1 + area2 - inter_area + 1e-6)
        return iou

def main():
    processor = YOLOPostProcessor(conf_threshold=0.5, iou_threshold=0.45)
    tracker = SimpleTracker(iou_threshold=0.3, max_age=5, min_hits=2)
    
    print("YOLO Post Processor initialized")
    print(f"  - Confidence threshold: {processor.conf_threshold}")
    print(f"  - IOU threshold (NMS): {processor.iou_threshold}")
    print(f"  - Tracker max age: {tracker.max_age}")

if __name__ == '__main__':
    main()
