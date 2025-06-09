#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os

def filter_map():
    # Percorsi relativi (usa il nome della tua mappa)
    original_map_path = os.path.join(os.path.dirname(__file__), '../map/map3_cropped.pgm')
    output_dir = os.path.join(os.path.dirname(__file__), '../map')
    
    # Carica la mappa mantenendo le sfumature di grigio
    map_img = cv2.imread(original_map_path, cv2.IMREAD_GRAYSCALE)
    if map_img is None:
        rospy.logerr("Mappa non trovata in: %s", original_map_path)
        return

    # 1. Filtro per rimuovere pixel isolati (salt-pepper noise)
    kernel = np.ones((5, 5), np.uint8)
    
    # Apertura morfologica SOLO per aree bianche (ostacoli isolati)
    _, white_mask = cv2.threshold(map_img, 254, 255, cv2.THRESH_BINARY)
    cleaned_white = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
    
    # Apertura morfologica SOLO per aree nere (spazi liberi isolati)
    _, black_mask = cv2.threshold(map_img, 1, 255, cv2.THRESH_BINARY_INV)
    cleaned_black = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)
    
    # 2. Ricostruzione della mappa preservando i grigi
    # Maschera delle aree modificate
    modified_areas = cv2.bitwise_or(cleaned_white, cleaned_black)
    
    # Aree bianche pulite
    final_white = np.where(cleaned_white == 255, 255, map_img)
    
    # Aree nere pulite
    final_map = np.where(cleaned_black == 255, 0, final_white)
    
    # 3. Salva la mappa filtrata
    output_path = os.path.join(output_dir, "map3_filtered.pgm")
    cv2.imwrite(output_path, final_map)
    rospy.loginfo(f"Mappa filtrata (con grigi) salvata in: {output_path}")

if __name__ == "__main__":
    rospy.init_node("map_filter")
    filter_map()
