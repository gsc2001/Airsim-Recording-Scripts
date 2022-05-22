import airsim
import numpy as np
import meshio

client = airsim.MultirotorClient()
client.reset()
client.confirmConnection()

meshes = client.simGetMeshPositionVertexBuffers()

for m in meshes:
	vertex_list = np.array(m.vertices, dtype=np.float32)
	indices = np.array(m.indices, dtype=np.float32)

	num_vertices = int(len(vertex_list)/3)
	num_indices = len(indices)

	vertices_reshaped = vertex_list.reshape((num_vertices, 3))
	indices_reshaped = indices.reshape((int(num_indices/3), 3))

	# save mesh using meshio
	cells = [("triangle", indices_reshaped)]
	meshio.write_points_cells("SavedStaticMeshes/"+m.name+".vtu",vertices_reshaped, cells)
	
	print("saved mesh : " + m.name)
