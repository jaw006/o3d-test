import sys
import open3d as o3d
import numpy as np

print('Number of arguments:', len(sys.argv), 'arguments.')
print('Argument List:', str(sys.argv))

if len(sys.argv) != 3:
	print("usage: py image_to_array.py <input_file> <output_csv>")
	exit

input_file = sys.argv[1]
output_csv = sys.argv[2]

# Extract image as array
img = o3d.io.read_image(input_file)
img_array = np.asarray(img)
output = np.asarray(img_array[288]) # capture middle frame

#np.transpose(output)
#output = output.transpose()

np.savetxt(output_csv, output, delimiter=',', newline='\n')
print(len(img_array))

# print('img_array')
# for row in img_array:
# 	print(row)

