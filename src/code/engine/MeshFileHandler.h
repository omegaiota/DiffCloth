//
// Created by Yifei Li on 2/26/21.
// Email: liyifei@csail.mit.edu
//

#ifndef OMEGAENGINE_MESHFILEHANDLER_H
#define OMEGAENGINE_MESHFILEHANDLER_H

#include "../simulation/Triangle.h"
#include "Macros.h"
#include "UtilityFunctions.h"
#include <iomanip>

class MeshFileHandler {
public:
	static std::vector<Vec3d> loadPosFile_txt(std::string fileName) {
		std::vector<Vec3d> pos;
		std::string line;

		std::string inputFolder = std::string(SOURCE_PATH) + "/src/assets/meshes/";
		std::ostringstream fNameStream;
		fNameStream << inputFolder << fileName;
		std::printf("Loading pos file from %s...\n", fNameStream.str().c_str());

		std::ifstream myfile(fNameStream.str().c_str());
		if (myfile.is_open()) {
			while (getline(myfile, line)) {
				bool foundFirstSpace = false;
				int firstSpace = 0;
				int secondSpace = 0;
				for (int i = 0; i < line.length(); i++) {
					if (line[i] == ' ') {
						if (!foundFirstSpace) {
							firstSpace = i;
							foundFirstSpace = true;
						} else {
							secondSpace = i;
							break;
						}
					}
				}

				Vec3d pos_i;
				pos_i[0] = std::atof(line.substr(0, firstSpace).c_str());
				pos_i[1] = std::atof(
						line.substr(firstSpace + 1, secondSpace - firstSpace - 1).c_str());
				pos_i[2] = std::atof(
						line.substr(secondSpace + 1, line.length() - secondSpace - 1)
								.c_str());
				pos.emplace_back(pos_i);
			}
			myfile.close();
			//        std::printf("success!\n");
		} else {
			std::printf("WARNING: POSITION FILE %s doesn't exist!\n",
					fNameStream.str().c_str());
		}

		return pos;
	}
	static void loadOBJFile(const char *filename, std::vector<Vec3d> &posVec,
			std::vector<Vec3i> &triVec) {
		FILE *file = fopen(filename, "rb");
		if (!file) {
			std::cerr << "file not exist:" << filename << std::endl;
			return;
		}

		bool has_normal = false;
		bool has_texture = false;
		char line_buffer[2000];
		while (fgets(line_buffer, 2000, file)) {
			char *first_token = strtok(line_buffer, "\r\n\t ");
			if (!first_token || first_token[0] == '#' || first_token[0] == 0)
				continue;

			switch (first_token[0]) {
				case 'v': {
					if (first_token[1] == 'n') {
						strtok(nullptr, "\t ");
						strtok(nullptr, "\t ");
						strtok(nullptr, "\t ");
						has_normal = true;
					} else if (first_token[1] == 't') {
						strtok(nullptr, "\t ");
						strtok(nullptr, "\t ");
						has_texture = true;
					} else {
						double x = atof(strtok(nullptr, "\t "));
						double y = atof(strtok(nullptr, "\t "));
						double z = atof(strtok(nullptr, "\t "));

						posVec.emplace_back(x, y, z);
					}
				} break;
				case 'f': {
					//        Triangle tri;
					int vIdx[3];
					char *data[30];
					int n = 0;
					while ((data[n] = strtok(nullptr, "\t \r\n")) != nullptr) {
						if (strlen(data[n]))
							n++;
					}

					for (int t = 0; t < (n - 2); ++t) {
						if ((!has_texture) && (!has_normal)) {
							vIdx[0] = atoi(data[0]) - 1;
							vIdx[1] = atoi(data[1]) - 1;
							vIdx[2] = atoi(data[2]) - 1;
						} else {
							const char *v1;
							for (int i = 0; i < 3; i++) {
								// vertex ID
								if (i == 0)
									v1 = data[0];
								else
									v1 = data[t + i];

								vIdx[i] = atoi(v1) - 1;
							}
						}
						triVec.emplace_back(vIdx[0], vIdx[1], vIdx[2]);
					}
				}
			}
		}

		fclose(file);

		std::printf("Finished loading obj file %s: vertex %zu triangles: %zu\n",
				filename, posVec.size(), triVec.size());
	}

	static void saveOBJFile(std::string filename, VecXd x_n,
			std::vector<Vec3i> &triangles) {
		checkFolderExistsAndCreate(OUTPUT_PARENT_FOLDER);
		std::ofstream os(filename + ".obj");

		if (!os) {
			std::cerr << "file not exist" << std::endl;
			return;
		}

		for (int i = 0; i < x_n.rows() / 3; i++) {
			Vec3d pos = x_n.segment(i * 3, 3);

			os << "v " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
		}

		for (std::size_t i = 0; i < triangles.size(); ++i) {
			Vec3i idx = triangles[i];

			os << "f " << idx[0] + 1 << " " << idx[1] + 1 << " " << idx[2] + 1
			   << std::endl;
		}
		os.close();
	}

	static void saveOBJFile(std::string filename, std::vector<Vec3d> x_n,
			std::vector<Vec3i> &triangles) {
		checkFolderExistsAndCreate(OUTPUT_PARENT_FOLDER);
		std::ofstream os(filename + ".obj");

		if (!os) {
			std::cerr << "file not exist" << std::endl;
			return;
		}

		for (int i = 0; i < x_n.size(); i++) {
			Vec3d pos = x_n[i];

			os << "v " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
		}

		for (std::size_t i = 0; i < triangles.size(); ++i) {
			Vec3i idx = triangles[i];

			os << "f " << idx[0] + 1 << " " << idx[1] + 1 << " " << idx[2] + 1
			   << std::endl;
		}
		os.close();
	}

	static void saveOBJFile(std::string filename, VecXd x_n,
			std::vector<Triangle> &triangles) {
		checkFolderExistsAndCreate(OUTPUT_PARENT_FOLDER);
		std::ofstream os(filename + ".obj");

		if (!os) {
			std::cerr << "file not exist" << std::endl;
			return;
		}

		for (int i = 0; i < x_n.rows() / 3; i++) {
			Vec3d pos = x_n.segment(i * 3, 3);

			os << "v " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
		}

		for (std::size_t i = 0; i < triangles.size(); ++i) {
			Vec3i idx = triangles[i].getIdxVector();

			os << "f " << idx[0] + 1 << " " << idx[1] + 1 << " " << idx[2] + 1
			   << std::endl;
		}
		os.close();
	}

	static void saveOBJFile(std::string filename, VecXd x_n, VecXd normals,
			std::vector<Triangle> &triangles) {
		checkFolderExistsAndCreate(OUTPUT_PARENT_FOLDER);
		std::ofstream os(filename + ".obj");

		if (!os) {
			std::cerr << "file not exist:" << filename + ".obj" << std::endl;
			return;
		}

		for (int i = 0; i < x_n.rows() / 3; i++) {
			Vec3d pos = x_n.segment(i * 3, 3);

			os << "v " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
		}

		for (std::size_t i = 0; i < triangles.size(); ++i) {
			Vec3i idx = triangles[i].getIdxVector();

			os << "f " << idx[0] + 1 << " " << idx[1] + 1 << " " << idx[2] + 1
			   << std::endl;
		}

		for (int i = 0; i < normals.rows() / 3; i++) {
			Vec3d n = normals.segment(i * 3, 3);

			os << "vn " << n[0] << " " << n[1] << " " << n[2] << std::endl;
		}
		os.close();
	}

	static void exportMeshPos(VecXd &x, std::string fileName) {
		std::printf("[exportMeshPos]\n");
		std::ofstream myfile;
		checkFolderExistsAndCreate(OUTPUT_PARENT_FOLDER);

		std::ostringstream fNameStream;
		fNameStream << OUTPUT_PARENT_FOLDER << fileName << ".txt";
		std::printf("begin to save current mesh position to: %s....",
				fNameStream.str().c_str());
		myfile.open(fNameStream.str());

		int precision = 3;
		for (int i = 0; i < x.rows() / 3; i++) {
			Vec3d pos = x.segment(i * 3, 3);
			myfile << std::fixed << std::setprecision(precision) << pos[0] << " "
				   << pos[1] << " " << pos[2] << "\n";
		}
		myfile.close();
	}
};

#endif // OMEGAENGINE_MESHFILEHANDLER_H
