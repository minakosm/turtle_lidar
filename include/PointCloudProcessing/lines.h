#ifndef LINES_H
#define LINES_H

#include <ostream>
#include <eigen3/Eigen/Dense>

//STRUCT LINES
struct Lines{
    Eigen::MatrixXf linesMatrix;
    Eigen::MatrixXf linesMedianMatrixX;
    Eigen::MatrixXf linesMedianMatrixY;
    Eigen::VectorXi sizes;

    Lines(int segments, int bins){
        linesMatrix = Eigen::MatrixXf::Zero(2*(bins-1), segments);
        linesMedianMatrixX = Eigen::MatrixXf::Zero((bins-1), segments);
        linesMedianMatrixY = Eigen::MatrixXf::Zero((bins-1), segments);
        sizes = Eigen::VectorXi::Zero(segments);
    }
};

//STRUCT SEGMENT-LINES
struct SegmentLines{
    Eigen::VectorXf linesMatrix;
    Eigen::VectorXf linesMedianMatrixX;
    Eigen::VectorXf linesMedianMatrixY;
    int size;

    SegmentLines(int bins){
        linesMatrix = Eigen::VectorXf::Zero(2*(bins-1));
        linesMedianMatrixX = Eigen::VectorXf::Zero(bins-1);
        linesMedianMatrixY = Eigen::VectorXf::Zero(bins-1);
        size = 0;
    }

    SegmentLines(const Lines &lines, int segments){
        linesMatrix = lines.linesMatrix.col(segments);
        linesMedianMatrixX = lines.linesMedianMatrixX.col(segments);
        linesMedianMatrixY = lines.linesMedianMatrixY.col(segments);
        size = lines.sizes(segments);
    }
};

std::ostream &operator<<(std::ostream &os, SegmentLines const &line){
    return os << "SegmentLines Lines Matrix =\n" << line.linesMatrix << std::endl << std::endl << "SegmentLine Lines MedianX =\n"
               << line.linesMedianMatrixX << std::endl << std::endl << "SegmentLines Lines MedianY =\n" << line.linesMedianMatrixY
               << std::endl << std::endl << "SegmentsLines size =\n" << line.size << std::endl << std::endl;
}

std::ostream &operator<<(std::ostream &os, Lines const &lines){
    return os << "Lines Matrix =\n" << lines.linesMatrix << std::endl << std::endl
              << "Lines MedianX =\n"<< lines.linesMedianMatrixX << std::endl << std::endl
              << "Lines MedianY = \n"<< lines.linesMedianMatrixY << std::endl << std::endl
              << "Lines size =\n" << lines.sizes << std::endl << std::endl;
}
#endif // LINES_H_INCLUDED