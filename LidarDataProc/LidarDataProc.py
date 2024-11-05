import os
import sys
path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(path)

# IMPORT EXTERN
import argparse
from typing import List

# IMPORT CLASS
from LidarPointArray import LidarPointArray
from GyroData import GyroData
# IMPORT UTIL METH
from write_data import *
from file_parser import *
from visualisation2d import *
from visualisation3d import *
from data_stabilisation import stabilise_lidar_array
from data_interpr import shape_interpr
from data_filter import filter_lidar_data
from line_generator import wave_clustering, line_2d_generate, barycentre_cluster
from point_movement_line import point_movement_line, find_direction_waves, wave_cluster_timelapse_generator
from visualisationStat import polar_angle, wave_height
from SimulatedSea import SimulatedSea

""" MAIN PYTHON SCRIPT.

Refer to the README for more information
"""

def main():
    # argument parsing
    parser = argparse.ArgumentParser(
        prog="LIDAR Data Processing",
        description="Process LIDAR wave surface data"
    )

    ### FILE PATH ARGS ###
    # Process LIDAR .pcap Data File
    parser.add_argument(
        "--lidar_vel",
        nargs=2,
        help="read Velodyne lidar .pcap",
        metavar=("LIDAR_FILE_PATH", "NUM_FRAME_TO_EXTRACT")
    )
    parser.add_argument(
        "--lidar_ous",
        nargs=3,
        help="read Ouster lidar .pcap",
        metavar=("LIDAR_FILE_PATH", "JSON_META_FILE_PATH", "NUM_FRAME_TO_EXTRACT")
    )
    parser.add_argument(
        "--simu",
        nargs=2,
        help="Generate a simulated sea",
        metavar=("SEA_TYPE", "NBR_FRAMES")
    )
    # Process GYRO .csv Data File
    parser.add_argument(
        "--gyro",
        nargs=1,
        help="read IMU csv file",
        metavar=("CSV_FILE_PATH",)
    )

    ### DATA PROCESSING ARGS ###
    # Process GYRO .csv Data File
    parser.add_argument(
        "--corr",
        nargs=1,
        help="correct the point cloud with IMU data (Yaw, Pitch, Roll)",
        metavar=("YPR_OPTION",)
    )
    parser.add_argument(
        "--prefilter",
        nargs=1,
        help="filter cloud point before correcting the data",
        metavar=("JSON_FILE_PATH",)
    )
    parser.add_argument(
        "--postfilter",
        nargs=1,
        help="filter cloud point after correcting the data",
        metavar=("JSON_FILE_PATH",)
    )

    ### DATA DISPLAY TYPE
    # Process LIDAR .pcap Data File
    parser.add_argument(
        "--display",
        nargs=1,
        help="display data : pc (point cloud), mesh (mesh generation), hex2d (hex top view), barycentre (barycentre of knn clusterisation of the pc), linebary (line of barycentre movement), wavedir (estimated direction of the wave), wavepolar (polar of the estimated direction of the wave), waveheight (average height of each cluster)",
        metavar=("DISPLAY_TYPE",)
    )

    # args
    args = parser.parse_args()

    # VARS
    array_lidar: List[LidarPointArray] = []
    array_gyro: List[GyroData] = []

    if args.lidar_vel:
        array_lidar = parse_lidar_vel_file_into_array(args.lidar_vel[0], args.lidar_vel[1])
    if args.lidar_ous:
        array_lidar = parse_lidar_ous_file_into_array(args.lidar_ous[0], args.lidar_ous[1], args.lidar_ous[2])
    if args.simu:
        simu = SimulatedSea(args.simu[0], args.simu[1])
        array_lidar = simu.get_array_lidar()

    if args.gyro:
        array_gyro = parse_gyro_file_data(args.gyro[0])

    if args.prefilter:
        filter_lidar_data(array_lidar, args.prefilter[0])

    if args.corr:
        if not args.gyro:
            print("ERROR : IMU data not found") 
            print("USE --gyro [path] IF YOU WANT TO CORRECT DATA!") 
            exit(1) # error
        array_lidar = stabilise_lidar_array(array_lidar, array_gyro, args.corr[0])

    if args.postfilter:
        filter_lidar_data(array_lidar, args.postfilter[0])

    if args.display:
        if args.display[0]=="pc":
            display_anim_point_array(array_lidar)
        elif args.display[0]=="mesh":
            meshs = []
            point_cloid = []
            meshs, point_cloid = shape_interpr(array_lidar)
            display_anim_mesh(meshs, point_cloid)
        elif args.display[0]=="hex2d":
            hex2dAnimates(array_lidar)
        elif args.display[0]=="wave2d":
            lines, points = line_2d_generate(array_lidar)
            dt_interval = array_lidar[1].timestamp - array_lidar[0].timestamp
            wave_line_anim(points, lines, dt_interval)
        elif args.display[0]=="barycentre":
            points, clusters = barycentre_cluster(array_lidar)
            dt_interval = array_lidar[1].timestamp - array_lidar[0].timestamp
            barycentre_anim(clusters, points, dt_interval)
        elif args.display[0]=="linebary":
            points, clusters = barycentre_cluster(array_lidar)
            line_wave = point_movement_line(points)
            dt_interval = array_lidar[1].timestamp - array_lidar[0].timestamp
            barycentre_anim_plus_line_wave(clusters, points, line_wave, dt_interval)
        elif args.display[0]=="wavedir":
            points, clusters = barycentre_cluster(array_lidar)
            line_wave = point_movement_line(points)
            coef_moy, coefs = find_direction_waves(line_wave)
            dt_interval = array_lidar[1].timestamp - array_lidar[0].timestamp
            barycentre_anim_line_wave_compass(clusters, points, line_wave, coef_moy, dt_interval)
        elif args.display[0]=="wavepolar":
            waves_clusters = wave_clustering(array_lidar)
            timelapses = wave_cluster_timelapse_generator(waves_clusters)
            timestamps = [array.timestamp for array in array_lidar]
            polar_angle(timestamps, timelapses)
        elif args.display[0]=="waveheight":
            waves_clusters = wave_clustering(array_lidar)
            timelapses = wave_cluster_timelapse_generator(waves_clusters)
            timestamps = [array.timestamp for array in array_lidar]
            wave_height(timestamps, timelapses)
        else:
            print("ERROR: Wrong parameter for display")
            exit(1)
    else:
        print("You didn't display anything, use --display if it's not intended!")

if __name__ == '__main__':
    # Run main entry point only if the script was run as a program
    # and not if it was merely imported
    main()