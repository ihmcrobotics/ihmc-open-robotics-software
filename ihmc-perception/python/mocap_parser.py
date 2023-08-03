import numpy as np
import matplotlib.pyplot as plt

def convert_file(name):
	f = open(name + '.csv', 'r')
	f_csv = open(name + '_Extracted.csv', 'w+')
	f_format = open(name + '_Format.txt', 'w+')

	start = False

	for l in f:

		values = l.strip().split(',')
		# print(len(values))

		if start == True:
			for i in range(1, len(values)):
				if values[i] == '':
					values[i] = '0'
				f_csv.write("{}\t".format(values[i]))
			f_csv.write('\n')
		else:
			for i in range(len(values)):
				f_format.write("{}\t".format(values[i]))
			f_format.write('\n')

		if values[0] == "Frame":
			start = True

		print('Values:', len(values), values)

	# print()
	# print()

def plot_data(name):
	csv_name = name + '_Extracted.csv', 'w+'
	format_name = name + '_Format.txt', 'w+'

	# for l in open(format_name):
	# 	words = l.strip().split("\t")
	# 	if words[0] == "Name":
	# 		print(words)

	import os
	print(os.listdir('./'))

	data = np.genfromtxt(csv_name, delimiter='\t')

	print(data.shape)

	t = np.linspace(0, data.shape[0], data.shape[0])



	# fig = plt.figure(figsize=(10,10))
	# plt.plot(data[:,1], data[:,2], 'ro')
	# plt.show()

def load_data(path, start=0, end=2000, verbose=False):
	f = open(path, 'r')

	data = []

	for i, line in enumerate(f):

		if i == 3:
			print('----------------------------------------------------------------------------------')
			print(line)
			print('----------------------------------------------------------------------------------')

		if i > 6 and i < end:

			words = line.split(',')

			for j in range(len(words)):
				if words[j] == '':
					words[j] = '0'

			if verbose == True:
				print('----------------------------------------------------------------------------------')
				print("Line: {} {}".format(i, len(words)))
				print(line)
				print('----------------------------------------------------------------------------------')

			data.append(np.array([	float(words[1]),
									float(words[2]), float(words[3]), float(words[4]),
									float(words[5]), float(words[6]), float(words[7]),
									float(words[8]), float(words[9]), float(words[10])]))

	matrix = np.vstack(data)

	print(matrix.shape)

	return matrix

def load_se_data(path, start=0, end=2000, verbose=False):
	f = open(path, 'r')

	data = []

	for i, line in enumerate(f):

		if i == 0:
			print('----------------------------------------------------------------------------------')
			print(line)
			print('----------------------------------------------------------------------------------')

		if i > start and i < end:

			words = line.split(',')

			for j in range(len(words)):
				if words[j] == '':
					words[j] = '0'

			if verbose == True:
				print('----------------------------------------------------------------------------------')
				print("Line: {} {}".format(i, len(words)))
				print(line)
				print('----------------------------------------------------------------------------------')

			data.append(np.array([	float(words[0]),
									float(words[1]), float(words[2]), float(words[3])]))

	matrix = np.vstack(data)

	print(matrix.shape)

	return matrix


def plot_marker(data, se_data, marker_id):
	
	f, ax = plt.subplots(3, 1, figsize=(15,8))

	ax[0].set_title("Pelvis Position (X)")
	ax[1].set_title("Pelvis Position (Y)")
	ax[2].set_title("Pelvis Position (Z)")

	# ax[0].plot(data[:,0], data[:,marker_id * 3 + 1], 'b-')
	# ax[1].plot(data[:,0], data[:,marker_id * 3 + 2], 'b-')
	# ax[2].plot(data[:,0], data[:,marker_id * 3 + 3], 'b-')
	
	
	ax[0].plot(se_data[:,0], se_data[:,1], 'r-')
	ax[1].plot(se_data[:,0], se_data[:,2], 'r-')
	ax[2].plot(se_data[:,0], se_data[:,3], 'r-')

	plt.show()

def plot_all(data):
	
	f, ax = plt.subplots(data.shape[1], 1, figsize=(10,8))

	
	for i in range(data.shape[1]):
		ax[i].plot(data[:,0], data[:,i], 'b-')

	plt.show()

def plot_marker_3d(data, se_data, marker_id):
	
	ax = plt.axes(projection='3d')

	
	ax.plot(data[:,marker_id * 3 + 1], data[:,marker_id * 3 + 2], data[:,marker_id * 3 + 3], 'r-')

	ax.set_xlim(2,6)
	ax.set_ylim(0,4)
	ax.set_zlim(-2,2)

	ax.plot(se_data[:,1], se_data[:,2], se_data[:,3], 'b-')

	plt.show()

if __name__ == "__main__":
	data = load_data('/home/bmishra/Workspace/Data/Atlas_Logs/MotionCapCsv/IHMC_Run_3.csv', start=0, end=20000, verbose=False)
	se_data = load_se_data('/home/bmishra/Workspace/Data/Atlas_Logs/Logs/Run_1/Export/data.scs2.csv', start=0, end=40000, verbose=False)

	plot_marker(data, se_data, 0)

