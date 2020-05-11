#!/usr/bin/env python2

import sys
import os.path
from datetime import datetime
import numpy as np

help = """Script convert PTX point cloud data to PCD for Point Cloud Library.
Usage:
    ptx2pcd.py input.ptx
    ptx2pcd.py ~/ptx_files_dir
    ptx2pcd.py -h|--help
Output:
    [original_filename]_bin.pcd
"""


def main():
    if '--help' in sys.argv or '-h' in sys.argv:
        print help
        sys.exit(0)
    if len(sys.argv) != 2:
        print >> sys.stderr, 'Too many parameters.'
        print help
        sys.exit(-1)
    if os.path.isdir(sys.argv[1]):
        for root, dirs, files in os.walk(sys.argv[1]):
            for file in files:
                filename = os.path.join(root, file)
                if is_vailed_file(filename):
                    process_file(filename)
    elif os.path.isfile(sys.argv[1]):
        process_file(sys.argv[1])
    else:
        print >> sys.stderr, 'Not a file or directory', sys.argv[1]
        sys.exit(-1)


def is_vailed_file(filename):
    if filename.endswith('.ptx'):
        return True
    return False


def process_line(line, transform):
    vals = np.array(map(float, str(line).split(" ")[:3]) + [1])
    # print "vals:", vals
    # print "tf:", transform
    vals2 = np.squeeze(np.asarray(vals))
    transform2 = np.squeeze(np.asarray(transform))

    pos = vals2.dot(transform2)
    return "{0} {1} {2}".format(pos[0], pos[1], pos[2])


def process_file(filename):
    print "start: ", datetime.now(), filename
    with open(filename, 'r') as ptx, open(filename + ".pcd", 'w') as pcd:
        col = int(ptx.readline())
        row = int(ptx.readline())
        # transformation matrix for view point
        view_point_tm = np.zeros([4, 3])
        # print ptx.readline().split(" ")
        view_point_tm[3] = map(float, ptx.readline().rstrip().split(" "))
        view_point_tm[0] = map(float, ptx.readline().rstrip().split(" "))
        view_point_tm[1] = map(float, ptx.readline().rstrip().split(" "))
        view_point_tm[2] = map(float, ptx.readline().rstrip().split(" "))
        cloud_tm = np.zeros([4, 3])  # transformation matrix for cloud
        cloud_tm[0] = map(float, ptx.readline().rstrip().split(" ")[:3])
        cloud_tm[1] = map(float, ptx.readline().rstrip().split(" ")[:3])
        cloud_tm[2] = map(float, ptx.readline().rstrip().split(" ")[:3])
        cloud_tm[3] = map(float, ptx.readline().rstrip().split(" ")[:3])
        # TODO: VIEW POINT: add quotanion
        pcd_head = '''# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {0}
HEIGHT {1}
VIEWPOINT {2} {3} {4} 1 0 0 0
POINTS {5}
DATA ascii\n'''.format(col, row,
                       view_point_tm[3, 0],
                       view_point_tm[3, 1],
                       view_point_tm[3, 2],
                       col * row)
        pcd.writelines(pcd_head)
        count = 0
        line = ptx.readline()
        while line:
            pcd.write(process_line(line, cloud_tm) + "\n")
            if 0 == count % 1000000:
                print "@line", count
            count += 1
            line = ptx.readline()
    print "end  : ", datetime.now(), filename


if __name__ == '__main__':
    main()