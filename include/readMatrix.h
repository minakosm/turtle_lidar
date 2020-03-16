#include <iostream>
#include <fstream>
#include <vector>
#include <eigen3/Eigen/Dense>

template <class T> void read_matrix (std::string fileName, Eigen::Matrix <T, Eigen::Dynamic, Eigen::Dynamic> &outputMat) {
	std::fstream cin;
	cin.open(fileName.c_str());
	if (cin.fail()) {
		std::cerr << "Failed to open file: " << fileName << std::endl;
		cin.get(); 
	}
	std::string s;
	std::vector <std::vector <T> > matrix;
	while (getline(cin, s)) {
		std::stringstream input(s);
		T temp;
		std::vector <T> currentLine;
		while (input >> temp)
			currentLine.push_back(temp);
		matrix.push_back(currentLine);
	}
	outputMat.resize(matrix.size(),3);
	for(unsigned int i = 0; i < matrix.size(); i++) {
		std::vector <T> row = matrix.at(i);
		for(unsigned int j = 0; j < row.size(); j++)
			outputMat(i,j) = row.at(j);
	}
}
