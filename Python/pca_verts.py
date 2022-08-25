import numpy as np
from sklearn.decomposition import PCA
import struct
import math
import os

bone_ids = [0]#[i for i in range(10)] + [i for i in range(33 , 37)]+ [i for i in range(60, 69)]
outputs_dir = "D:/Unity/Unity 2021 Editor Test/Assets/outputs/vertex_data/" # sprint1_subject2_output.txt
newfiles_dir = "D:/Unity/Unity 2021 Editor Test/Python/pyoutputs/capsule/" # sprint1_subject2_output.txt

NUM_BYTES_IN_FLOAT  = 4

def get_np_array(bone_id):
    filename = outputs_dir + 'bone' + str(bone_id) + '_verts.bin'
    # file_size = os.path.getsize(filename)
    # print("File Size is :" +  str(file_size) +  "bytes")
    arr = []
    with open(filename, mode="rb") as file:
        while True:
            file_buffer = file.read(NUM_BYTES_IN_FLOAT * 3)
            if file_buffer == '': #file.tell() == os.fstat(file.fileno()).st_size:
                break
            xyz = struct.unpack('f' * 3, file_buffer)
            arr.append(list(xyz))
            # np.append(arr, list(xyz), axis=0)
    return np.array(arr)

def get_np_array_txt(bone_id):
    filename = outputs_dir + 'bone' + str(bone_id) + '_verts.txt'
    arr = []
    with open(filename) as f:
        # get rid of opening line
        f.readline()
        contents = f.readlines()
        for line in contents:
            values = [float(s.strip()) for s in line.split(",")]
            # np.append(arr, [values], axis=0)
            arr.append(values)
    return np.array(arr)

def get_max_dist_apart(proj_verts):
    val = float('-inf')
    for i in range(len(proj_verts)):
        for j in range(i + 1, len(proj_verts)):
            val = max(val, magnitude(proj_verts[i] - proj_verts[j]))
    return val

def project_verts_onto_axis(verts, point_a, point_b):
    proj_verts = []
    for i in xrange(verts.shape[0]):
        vert = verts[i]
        proj_vert = ClosestPointOnLine(point_a, point_b, vert)
        proj_verts.append(proj_vert)
    return proj_verts



def get_cost_of_radius(mean, largest_eigen, verts, radius):
    cur_cost = 0
    # should break if the radius is so low that all the vertices are outside
    should_break = True
    for i in xrange(verts.shape[0]):
        vert = verts[i]
        dist_to_axis = magnitude(vert - ClosestPointOnLine(mean, mean + largest_eigen, vert))
        should_break = should_break and dist_to_axis > radius
        cur_cost += (dist_to_axis - radius)**2
    return cur_cost, should_break


RADIUS_ITERATOR = .0001
def calc_height_and_rad(verts):
    mean = np.mean(verts, axis=0)
    pca = PCA(n_components = 3, svd_solver='full')
    pca.fit(verts)
    print("Covar:" , pca.get_covariance())
    largest_eigen = pca.components_[0]
    print("largest_eigen" ,largest_eigen)
    print("largest_eigen normalized" ,normalize(largest_eigen))

    second_eigen = pca.components_[1]

    proj_verts = project_verts_onto_axis(verts, mean, mean + largest_eigen)
    height = get_max_dist_apart(proj_verts)

    radius =  sum([magnitude(v - ClosestPointOnLine(mean, mean + largest_eigen, v)) for v in verts]) / len(verts)
    return height, radius

# https://gamedev.stackexchange.com/questions/72528/how-can-i-project-a-3d-point-onto-a-3d-line
def ClosestPointOnLine(a, b, p):
    ap = p-a
    ab = b-a
    result = a + dot(ap,ab)/dot(ab,ab) * ab
    return result

def dot(a, b):
    return sum([a[i] * b[i] for i in range(len(a))])

def cross(a, b):
    c = [a[1]*b[2] - a[2]*b[1],
         a[2]*b[0] - a[0]*b[2],
         a[0]*b[1] - a[1]*b[0]]

    return c

def magnitude(vec):
    return math.sqrt(sum(i ** 2 for i in vec))

def sqr_mag(vec):
    return sum(i ** 2 for i in vec)

def test():
    verts = get_np_array(8)
    print(calc_height_and_rad(verts))
    mean = np.mean(verts, axis=0)
    print("Mean: ", mean)
    pca = PCA(n_components = 3, svd_solver='full')
    pca.fit(verts)
    largest_eigen = pca.components_[0]
    print("Orientation: " , get_direction_and_orientation(mean,largest_eigen))

def normalize(vec):
    mag = magnitude(vec)
    return [v /mag for v in vec]

def get_direction_and_orientation(mean, largest_eigen):
    dir = 0
    # orient = (0, 0, 0, 1)
    dir_vec = normalize(largest_eigen)
    right_vec = (1, 0 , 0)
    a = cross(right_vec , dir_vec)
    # print("a", a)
    # a = cross(right_vec , dir_vec )
    orient = [a[0], a[1], a[2], 0]
    # orient[3] = dot(dir_vec, right_vec)
    orient[3] = math.sqrt(sqr_mag(dir_vec) * sqr_mag(right_vec)) + dot(dir_vec, right_vec)
    return normalize(orient)


    # up_vec = (0, 1, 0)
    # forward_vec = (0 , 0 ,1)


    # return dir, orient

def main():
    test()
    # for bone_id in bone_ids:
    #     bone_verts = get_np_array(bone_id)


if __name__ == "__main__":
    main()


