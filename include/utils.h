#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>

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
	outputMat.resize(matrix.size(),matrix.at(0).size());
	for(unsigned int i = 0; i < matrix.size(); i++) {
		std::vector <T> row = matrix.at(i);
		for(unsigned int j = 0; j < row.size(); j++)
			outputMat(i,j) = row.at(j);
	}
}

template <class T> void read_vector (std::string fileName, Eigen::Matrix <T, Eigen::Dynamic, 1> &outputVec) {
	std::fstream cin;
	cin.open(fileName.c_str());
	if (cin.fail()) {
		std::cerr << "Failed to open file: " << fileName << std::endl;
		cin.get(); 
	}
	std::string s;
	std::vector <T> vec;
	while (getline(cin, s)) {
		std::stringstream input(s);
		T temp;
		input >> temp;
		vec.push_back(temp);
	}
	outputVec.resize(vec.size());
	for(unsigned int i = 0; i < vec.size(); i++)
		outputVec(i) = vec.at(i);
}

template <class T> Eigen::Matrix <T, Eigen::Dynamic, 1> sliceMatrixRowwise(const Eigen::Matrix <T, Eigen::Dynamic, Eigen::Dynamic> &mat, const Eigen::Array <bool, Eigen::Dynamic, 1> &boolVec) {
    unsigned int outSize = (boolVec == 1).count();
    Eigen::Matrix <T, Eigen::Dynamic, Eigen::Dynamic> out(outSize, mat.cols());
    unsigned int counter = 0;
    if(boolVec.size() != mat.rows())
        std::cout << "Incompatible dimensions entered on sliceVector function!\n";
    for(unsigned int i = 0; i < mat.rows(); i++) {
        if(boolVec(i) == 1)
            out.row(counter++) = mat.row(i);
    }
    return out;
}

template <class T> Eigen::Matrix <T, Eigen::Dynamic, 1> sliceVector(const Eigen::Matrix <T, Eigen::Dynamic, 1> &vec, const Eigen::Array <bool, Eigen::Dynamic, 1> &boolVec) {
    unsigned int outSize = (boolVec == 1).count();
    Eigen::Matrix <T, Eigen::Dynamic, 1> out(outSize);
    unsigned int counter = 0;
    if(boolVec.size() != vec.size())
        std::cout << "Incompatible dimensions entered on sliceVector function!\n";
    for(unsigned int i = 0; i < vec.size(); i++) {
        if(boolVec(i) == 1)
            out(counter++) = vec(i);
    }
    return out;
}

#endif // UTILS_H_INCLUDED
