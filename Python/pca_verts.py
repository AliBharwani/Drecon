import numpy as np
from sklearn.decomposition import PCA


bone_ids = [0]#[i for i in range(10)] + [i for i in range(33 , 37)]+ [i for i in range(60, 69)]
outputs_dir = "D:/Unity/Unity 2021 Editor Test/Assets/outputs/vertex_data/" # sprint1_subject2_output.txt


def get_np_array(bone_id):
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


for bone_id in bone_ids:
    arr = get_np_array(bone_id)
    print(arr)