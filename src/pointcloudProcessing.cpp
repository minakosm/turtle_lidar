#include "PointCloudProcessing/PoincloudProcessing.h"
#include "settings.h"
#include "PointCloudProcessing/lines.h"

#include <eigen3/Eigen/Dense>
#include <limits>
#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <map>

PointcloudProcessing::PointcloudProcessing(std::string pathToConfigFile) : baseFilter(segments, bins, pathToConfigFile),
                                                                           groundFilter(pathToConfigFile),
                                                                           lines(segments, bins){

x = std::make_unique<Eigen::VectorXf>();
y = std::make_unique<Eigen::VectorXf>();
z = std::make_unique<Eigen::VectorXf>();
xBuffer = std::make_unique<Eigen::VectorXf>();
yBuffer = std::make_unique<Eigen::VectorXf>();
zBuffer = std::make_unique<Eigen::VectorXf>();

azim = std::make_unique<Eigen::VectorXf>();
r = std::make_unique<Eigen::VectorXf>();
azimBuffer = std::make_unique<Eigen::VectorXf>();
rBuffer = std::make_unique<Eigen::VectorXf>();

intensities = std::make_unique<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>>();
intensitiesBuffer = std::make_unique<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>>();
intensitiesFiltered = std::make_unique<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>>();

cart = std::make_unique<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>();
}



PointcloudProcessing::PointcloudProcessing(int pclSize, std::string pathToConfigFile) : baseFilter(segments, bins, pathToConfigFile),
                                                                           groundFilter(pathToConfigFile),
                                                                           lines(segments, bins){

x = std::make_unique<Eigen::VectorXf>(pclSize);
y = std::make_unique<Eigen::VectorXf>(pclSize);
z = std::make_unique<Eigen::VectorXf>(pclSize);
xBuffer = std::make_unique<Eigen::VectorXf>(pclSize);
yBuffer = std::make_unique<Eigen::VectorXf>(pclSize);
zBuffer = std::make_unique<Eigen::VectorXf>(pclSize);

azim = std::make_unique<Eigen::VectorXf>(pclSize);
r = std::make_unique<Eigen::VectorXf>(pclSize);
azimBuffer = std::make_unique<Eigen::VectorXf>(pclSize);
rBuffer = std::make_unique<Eigen::VectorXf>(pclSize);

intensities = std::make_unique<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>>(pclSize);
intensitiesBuffer = std::make_unique<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>>(pclSize);
intensitiesFiltered = std::make_unique<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>>(pclSize);

cart = std::make_unique<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>(pclSize, 3);
}

void PointcloudProcessing::resizeCoordinates(int pclSize){
    xBuffer->resize(pclSize);
    yBuffer->resize(pclSize);
    zBuffer->resize(pclSize);
    intensitiesBuffer->resize(pclSize);
    azimBuffer->resize(pclSize);
    rBuffer->resize(pclSize);
}

void PointcloudProcessing::printPointcloudSize(){
    std::cout << "Pointcloud Contains " << x->size() << " points\n";
}

void PointcloudProcessing::printSettings(){
    std::cout << "Base Filter Settings\n" << baseFilter
              << "Ground Filter Settings\n" << groundFilter;
}

// GETTERS

Eigen::VectorXf PointcloudProcessing::getX(){
    return (*x);
}

Eigen::VectorXf PointcloudProcessing::getY(){
    return (*y);
}

Eigen::VectorXf PointcloudProcessing::getZ(){
    return (*z);
}

Eigen::VectorXf PointcloudProcessing::getAzim(){
    return (*azim);
}

Eigen::VectorXf PointcloudProcessing::getR(){
    return (*r);
}

Eigen::Matrix<uint16_t, Eigen::Dynamic, 1> PointcloudProcessing::getIntensities(){
    return (*intensities);
}

Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> PointcloudProcessing::getCart(){
    return (*cart);
}

int PointcloudProcessing::getNonGroundPoints(){
    return cart->rows();
}

Eigen::Matrix<uint16_t, Eigen::Dynamic, 1> PointcloudProcessing::getIntensitiesFiltered(){
    return (*intensitiesFiltered);
}

BaseFilter PointcloudProcessing::getBaseFilterSettings(){
    return baseFilter;
}

GroundFilter PointcloudProcessing::getGroundFilterSettings(){
    return groundFilter;
}

// FUNCTIONS

void PointcloudProcessing::polarInit(){
    azimBuffer->resize(xBuffer->size());
    rBuffer->resize(rBuffer->size());

    for(int i = 0; i<xBuffer->size(); i++){
        (*azimBuffer)(i) = std::atan2((*yBuffer)(i), (*xBuffer)(i));
    }

    (*rBuffer).noalias() = (xBuffer->array().square() + yBuffer->array().square()).sqrt().matrix();
    prototypePointsMatrix = Eigen::MatrixXi::Constant(segments, bins, -1);
    if(baseFilter.filterEnabled == 0){
        partitionMatrix.resize(xBuffer->size(), Eigen::NoChange);
        groundArray = Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(xBuffer->size(), 1, false);
    }
}

bool PointcloudProcessing::filterPoints(const Eigen::Array <bool, Eigen::Dynamic, 1> &logicalVector){
    unsigned int size = (logicalVector == 0).count();
    std::cout << "logicalVectorSize = " << size << std::endl; 
    if(size == 0){
        return false;
    }

    x->resize(size);
    y->resize(size);
    z->resize(size);
    azim->resize(size);
    r->resize(size);
    intensities->resize(size);

    unsigned int counter = 0;
    for(unsigned int i=0; i < xBuffer->rows(); i++){
        if(logicalVector(i) == 0){
            (*x)(counter) = (*xBuffer)(i);
            (*y)(counter) = (*yBuffer)(i);
            (*z)(counter) = (*zBuffer)(i);
            (*azim)(counter) = (*azimBuffer)(i);
            (*r)(counter) = (*rBuffer)(i);
            (*intensities)(counter++) = (*intensitiesBuffer)(i);
        }
    }
    return true;
}

bool PointcloudProcessing::filter() {
	if(!filterPoints((((azimBuffer->array() > baseFilter.maxAzim).cast<int>() + (azimBuffer->array() < baseFilter.minAzim).cast<int>())*(int)baseFilter.filterAzim +
                      ((rBuffer->array() > baseFilter.maxRad).cast<int>() + (rBuffer->array() < baseFilter.minRad).cast<int>())*(int)baseFilter.filterRad +
                      ((zBuffer->array() > baseFilter.maxZ).cast<int>() + (zBuffer->array() < baseFilter.minZ).cast<int>())*(int)baseFilter.filterZ).cast<bool>()))
        return false; 
    partitionMatrix.resize(x->size(), Eigen::NoChange);
    groundArray = Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(x->size(), 1, false);
    return true;
}

void PointcloudProcessing::calculatePartitionMatrix(){
    minAzim = azim->minCoeff();
    maxAzim = azim->maxCoeff();
    minRad = r->minCoeff();
    maxRad = r->maxCoeff();
    shiftAngle = (maxAzim - minAzim) / ((float)segments);
    shiftRad = (maxRad - minRad) / ((float)bins);

    partitionMatrix.col(0).noalias() = ((azim->array() - minAzim) / shiftAngle).floor().cast<uint8_t>().matrix();
    partitionMatrix.col(1).noalias() = ((r->array() - minRad) / shiftRad).floor().cast<uint8_t>().matrix();

    partitionMatrix.col(0) = (partitionMatrix.col(0).array() > (segments - 1)).select(segments - 1, partitionMatrix.col(0));
    partitionMatrix.col(1) = (partitionMatrix.col(1).array() > (bins - 1)).select(bins - 1, partitionMatrix.col(1));

    partitionMatrix.col(0) = (partitionMatrix.col(0).array() < 0).select(0, partitionMatrix.col(0));
    partitionMatrix.col(1) = (partitionMatrix.col(1).array() < 0).select(0, partitionMatrix.col(1));
}

void PointcloudProcessing::calculatePrototypePointsMatrix(){
    Eigen::MatrixXf prototypePointsDistances = Eigen::MatrixXf::Constant(segments, bins, 100000.0f);
    for(int k = 0; k < z->size(); k++){
        int seg = partitionMatrix(k,0);
        int bin = partitionMatrix(k,1);
        if((*z)(k) < prototypePointsDistances(seg, bin)){
            prototypePointsDistances(seg, bin) = (*z)(k);
            prototypePointsMatrix(seg, bin) = k;
        }
    }

    checkPrototypePointsMatrix();
}

void PointcloudProcessing::checkPrototypePointsMatrix(){
    Eigen::Array <bool, Eigen::Dynamic, Eigen::Dynamic> flags = ((prototypePointsMatrix.array() != -1).cast<int>().rowwise().sum().array() == 1).cast<bool>();
    Eigen::Matrix <int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> prototypePointsMatrixCopy = prototypePointsMatrix;

    for(int k = 0; k < segments; k++){
        if(flags(k)){
            int nonZeroIndex = 0, pointIndex = 0;
            for(int j=0; j<bins; j++){
                if(prototypePointsMatrix(k,j) != -1){
                    nonZeroIndex = j;
                    pointIndex = prototypePointsMatrix(k,j);
                    break;
                }
            }
            float minDist = std::numeric_limits<float>::infinity();
            int closestPointIndex = 0;
            for(int i=0; i<segments; i++){
                for(int j=0; j<bins; j++){
                    if(prototypePointsMatrix(i,j) == -1 || prototypePointsMatrix(i,j) == pointIndex)
                        continue;
                    int currentIndex = prototypePointsMatrix(i,j);
                    float dist = (*r)(currentIndex)*(*r)(currentIndex)+(*r)(pointIndex)*(*r)(pointIndex)-2*(*r)(currentIndex)*(*r)(pointIndex)*std::cos((*azim)(currentIndex)-(*azim)(pointIndex))+(*z)(currentIndex)*(*z)(currentIndex)+(*z)(pointIndex)*(*z)(pointIndex)-2*(*z)(currentIndex)*(*z)(pointIndex);
                    if(dist < minDist){
                        minDist = dist;
                        closestPointIndex = currentIndex;
                    }

                }
            }

            prototypePointsMatrixCopy(k, ((nonZeroIndex+1) % bins)) == closestPointIndex;
        }
    }
    prototypePointsMatrix = prototypePointsMatrixCopy;
}

//GROUND LINES FIT 
void PointcloudProcessing::groundLinesFit(){
    for(int i = 0; i < prototypePointsMatrix.rows(); i++){
        SegmentLines line = segmentGroundLinesFit(i);
        lines.linesMatrix.col(i) = line.linesMatrix;
        lines.linesMedianMatrixX.col(i) = line.linesMedianMatrixX;
        lines.linesMedianMatrixY.col(i) = line.linesMedianMatrixY;
        lines.sizes(i) = line.size;
    }
}
//GETLINE
Eigen::VectorXf PointcloudProcessing::getLine(const Eigen::VectorXf &properties){

    /** line(0) = m
    *   line(1) = b 
    *   line(2) = RMSE
    */

    Eigen::VectorXf line(3);
    line(0) = properties(4) / properties(2);
    line(1) = properties(1) - line(0) * properties(0);
    line(2) = sqrtf(1 - (properties(4) / sqrtf(properties(2) * properties(3))) * (properties(4) / sqrtf(properties(2) * properties(3)))) * sqrtf(properties(3));
	return line;
}
//UPDATELINE


Eigen::VectorXf PointcloudProcessing::updateLine(float x, float y, const Eigen::VectorXf &properties){

/** properties(0) = mean(X)
 *  properties(1) = mean(Y)
 *  properties(2) = var(X)
 *  properties(3) = var(Y)
 *  properties(4) = cov(X,Y)
 *  properties(5) = n
**/
    Eigen::VectorXf newProperties(6);
    float dx = x - properties(0);
    float dy = y - properties(1);

    newProperties(2) = properties(2) + (properties(5)/(properties(5)+1) * dx * dx - properties(2)) / (properties(5)+1);
	newProperties(3) = properties(3) + (properties(5)/(properties(5)+1) * dy * dy - properties(3)) / (properties(5)+1);
	newProperties(4) = properties(4) + (properties(5)/(properties(5)+1) * dx * dy - properties(4)) / (properties(5)+1);

    newProperties(0) = properties(0) + dx/(properties(5)+1);
    newProperties(1) = properties(1) + dy/(properties(5)+1);
    newProperties(5) = properties(5) + 1;
    return newProperties;
}

//SEGMENTGROUNDLINESFIT
SegmentLines PointcloudProcessing::segmentGroundLinesFit(int segment) {
    Eigen::VectorXi prototypePointsVector = prototypePointsMatrix.row(segment);
    SegmentLines lineVector(bins);
    Eigen::VectorXf currentLineProperties = Eigen::Matrix <float, 6, 1>::Zero();
    if((prototypePointsVector.array() != -1).count() == 2) {
        int index1 = 0, index2 = 0;
        bool flag = false;
        for(int i = 0; i < prototypePointsVector.size(); i++) {
            if(prototypePointsVector(i) != -1 && flag == false) {
                index1 = prototypePointsVector(i);
                flag = true;   
            }
            else if(prototypePointsVector(i) != -1 && flag == true)
                index2 = prototypePointsVector(i);
        }
        currentLineProperties = updateLine((*r)(index1), (*z)(index1), currentLineProperties);
        Eigen::VectorXf line = getLine(updateLine((*r)(index2), (*z)(index2), currentLineProperties));
        lineVector.linesMatrix(0) = line(0);
        lineVector.linesMatrix(1) = line(1);
        lineVector.linesMedianMatrixX(0) = ((*r)(index1)+(*r)(index2)) / 2;
        lineVector.linesMedianMatrixY(0) = ((*z)(index1)+(*z)(index2)) / 2;
        lineVector.size = 1;
    } 
    else {
        int lastIndex = -1;
        Eigen::VectorXf previousLineParam(3);
        previousLineParam << 0.0, 10000.0f, 0.0;
        std::vector <int> Pl;
        Pl.reserve(bins);
        int i = 0;
        while(i < bins) {
            int point = prototypePointsVector(i);
            if(point != -1) {
                if(Pl.size() > 0) {
                    Eigen::VectorXf potentialLine = getLine(updateLine((*r)(point), (*z)(point), currentLineProperties));
                    if ((std::abs(potentialLine(0)) < groundFilter.mMax) && ((potentialLine(0) > groundFilter.mSmall) ||  ((potentialLine(1) < groundFilter.bMax) && (potentialLine(1) > groundFilter.bMin))) && (potentialLine(2) < groundFilter.maxRmse)) {
                        Pl.push_back(point);
                        currentLineProperties = updateLine((*r)(point), (*z)(point), currentLineProperties);
                        lastIndex = i;
                    }
                    else if (Pl.size() > 1) {
                        previousLineParam = getLine(currentLineProperties);
                        lineVector.linesMatrix(2*lineVector.size) = previousLineParam(0);
                        lineVector.linesMatrix(2*lineVector.size + 1) = previousLineParam(1);
                        lineVector.linesMedianMatrixX(lineVector.size) = ((*r)(Pl.front())+(*r)(Pl.back())) / 2;
                        lineVector.linesMedianMatrixY(lineVector.size) = ((*z)(Pl.front())+(*z)(Pl.back())) / 2;
                        lineVector.size++;
                        i = lastIndex-1;
                        Pl.erase(Pl.begin(), Pl.end());
                        currentLineProperties = Eigen::Matrix <float, 6, 1>::Zero();
                    }
                }
                else if((distPointFromLine((*r)(point),(*z)(point),previousLineParam) < groundFilter.dPrev) || lineVector.size == 0) {
                    lastIndex = i;
                    Pl.push_back(point);
                    currentLineProperties = updateLine((*r)(point), (*z)(point), currentLineProperties);
                }
            }
            i++;
        }
        if(Pl.size() > 1) {
            previousLineParam = getLine(currentLineProperties);
            lineVector.linesMatrix(2*lineVector.size) = previousLineParam(0);
            lineVector.linesMatrix(2*lineVector.size + 1) = previousLineParam(1);
            lineVector.linesMedianMatrixX(lineVector.size) = ((*r)(Pl.front())+(*r)(Pl.back())) / 2;
            lineVector.linesMedianMatrixY(lineVector.size) = ((*z)(Pl.front())+(*z)(Pl.back())) / 2;
            lineVector.size++;
        }
    }
    return lineVector;
}


//GROUND_CLASSIFIER
void PointcloudProcessing::groundClassifier() {
    Eigen::VectorXf paramM(r->rows());
    Eigen::VectorXf paramB(r->rows());
    #pragma omp parallel for ordered num_threads(NUM_OF_THREADS)
		for(int i = 0; i < r->rows(); i++) {
            int segment = partitionMatrix(i,0);
            SegmentLines lineVector(lines, segment);
            int lineSelection = 0;
            float minDist = ((*r)(i)-lineVector.linesMedianMatrixX(0)) * ((*r)(i)-lineVector.linesMedianMatrixX(0)) + ((*z)(i)-lineVector.linesMedianMatrixY(0)) * ((*z)(i)-lineVector.linesMedianMatrixY(0));
            for(int j = 1; j < lineVector.size; j++) {
                float dist = ((*r)(i)-lineVector.linesMedianMatrixX(j)) * ((*r)(i)-lineVector.linesMedianMatrixX(j)) + ((*z)(i)-lineVector.linesMedianMatrixY(j)) * ((*z)(i)-lineVector.linesMedianMatrixY(j));
                if(dist < minDist) {
                    minDist = dist;
                    lineSelection = j;
                }
			}
            //if(lineVector.size > 1)
			    //((Eigen::VectorXf::Constant(lineVector.size,(*r)(i))-lineVector.linesMedianMatrixX.segment(0,lineVector.size)).cwiseProduct(Eigen::VectorXf::Constant(lineVector.size,(*r)(i))-lineVector.linesMedianMatrixX.segment(0,lineVector.size)) + (Eigen::VectorXf::Constant(lineVector.size,(*z)(i))-lineVector.linesMedianMatrixY.segment(0,lineVector.size)).cwiseProduct(Eigen::VectorXf::Constant(lineVector.size,(*z)(i))-lineVector.linesMedianMatrixY.segment(0,lineVector.size))).minCoeff(&lineSelection);
			paramM(i) = lineVector.linesMatrix(2*lineSelection);
			paramB(i) = lineVector.linesMatrix(2*lineSelection+1);
        }
	groundArray = ((((-paramM).cwiseProduct((*r))+(*z)-paramB).cwiseAbs()).cwiseQuotient((paramM.cwiseProduct(paramM)+Eigen::VectorXf::Constant(r->rows(),1)).cwiseSqrt())).array() < groundFilter.dGround;
}


//DISTPOINTFROMLINE
inline float PointcloudProcessing::distPointFromLine(float x, float y, const Eigen::VectorXf &param){
    return std::abs(-param(0)* x + y - param(1)) / sqrt(param(0) * param(0) + 1);
}
//FILTERGROUND
bool PointcloudProcessing::filterGround(){
    unsigned int size = (groundArray == 0).count();
    if(size == 0){
        return false;
    }
    unsigned int counter = 0;
    cart->resize(size,3);
    intensitiesFiltered->resize(size);
    for(unsigned int i=0; i < x->rows(); i++){
        if (groundArray(i) == 0){
            (*cart)(counter, 0) = (*x)(i);
            (*cart)(counter, 1) = (*y)(i);
            (*cart)(counter, 2) = (*z)(i);
            (*intensitiesFiltered)(counter++) = (*intensities)(i);
        }
    }
    return true;
}

//CONDENSEDDISTANCES
void PointcloudProcessing::condensedDistances() {
    int N = cart->rows();
	condensedClusterDistances.resize((N * (N-1)) / 2);
	#pragma omp parallel for ordered shared(cart,condensedClusterDistances) num_threads(NUM_OF_THREADS)
            for(int i = 0; i < (N-1); i++)
                (condensedClusterDistances.segment(i*(N-1) - (i*(i-1))/2, N-1-i)).noalias() = (cart->bottomRows(N-i-1).rowwise() - cart->row(i)).array().square().rowwise().sum().matrix().cast<double>();
}

int PointcloudProcessing::pipeline(std::unique_ptr<Eigen::VectorXf> &X, 
                                   std::unique_ptr<Eigen::VectorXf> &Y, 
                                   std::unique_ptr<Eigen::VectorXf> &Z, 
                                   std::unique_ptr<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>> &intensities,
                                   Eigen::Matrix <float, Eigen::Dynamic, 2, Eigen::RowMajor> &conePos,
                                   int maxPointsProcessing, int timeoutProcessing) {
    // std::cout << "Pipeline started\n";
    auto a1 = std::chrono::steady_clock::now();
    this->xBuffer.swap(X);
    this->yBuffer.swap(Y);
    this->zBuffer.swap(Z);
    this->intensitiesBuffer.swap(intensities);
    if(xBuffer->size() == 0)
        return -1;
    // std::cout << "Pointcloud size before base filter = " << xBuffer->size() << std::endl;
    auto a2 = std::chrono::steady_clock::now();
    // std::cout << "Swap time in = " << std::chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "us\n";
    if(std::chrono::duration_cast<std::chrono::microseconds>(a2 - a1).count() > timeoutProcessing*1000)
        return -100;
    // std::cout << "Swapping completed\n";
    polarInit();
    a2 = std::chrono::steady_clock::now();
    // std::cout << "Polar init time in = " << std::chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "us\n";
    if(std::chrono::duration_cast<std::chrono::microseconds>(a2 - a1).count() > timeoutProcessing*1000)
        return -100;
    // std::cout << "Polar coordinates calculated completed\n";
    if(!filter())
        return -2;
    // std::cout << "Pointcloud size after base filter = " << x->size() << std::endl;
    a2 = std::chrono::steady_clock::now();
    // std::cout << "Base filter time in = " << std::chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "us\n";
    if(std::chrono::duration_cast<std::chrono::microseconds>(a2 - a1).count() > timeoutProcessing*1000)
        return -100;
    // std::cout << "Main filter applied\n";
    calculatePartitionMatrix();
    // std::cout << "minAzim = " << minAzim << "\nmaxAzim = " << maxAzim << std::endl;
    a2 = std::chrono::steady_clock::now();
    // std::cout << "calculatePartitionMatrix time in = " << std::chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "us\n";
    if(std::chrono::duration_cast<std::chrono::microseconds>(a2 - a1).count() > timeoutProcessing*1000)
        return -100;
    // std::cout << "Partition Matrix calculated\n";
    calculatePrototypePointsMatrix();
    a2 = std::chrono::steady_clock::now();
    // std::cout << "calculatePrototypePointsMatrix time in = " << std::chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "us\n";
    if(std::chrono::duration_cast<std::chrono::microseconds>(a2 - a1).count() > timeoutProcessing*1000)
        return -100;
    // std::cout << "Prototype Matrix calculated\n";
    groundLinesFit();
    a2 = std::chrono::steady_clock::now();
    // std::cout << "groundLinesFit time in = " << std::chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "us\n";
    if(std::chrono::duration_cast<std::chrono::microseconds>(a2 - a1).count() > timeoutProcessing*1000)
        return -100;
    // std::cout << "Ground lines fitted\n";
    groundClassifier();
    a2 = std::chrono::steady_clock::now();
    // std::cout << "groundClassifier time in = " << std::chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "us\n";
    if(std::chrono::duration_cast<std::chrono::microseconds>(a2 - a1).count() > timeoutProcessing*1000)
        return -100;
    // std::cout << "Ground points found\n";
    if(!filterGround())
        return -3;
    a2 = std::chrono::steady_clock::now();
    // std::cout << "filterGround time in = " << std::chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "us\n";
    if(std::chrono::duration_cast<std::chrono::microseconds>(a2 - a1).count() > timeoutProcessing*1000)
        return -100;
    // std::cout << "Pointcloud size after ground filter = " << cart->rows() << std::endl;
    if(cart->rows() > maxPointsProcessing)
        return -101;
    // std::cout << "Ground points filtered\n";
    a2 = std::chrono::steady_clock::now();
    
    std::cout << "Total pipeline in us: " << std::chrono::duration_cast<std::chrono::microseconds>(a2 - a1).count() << std::endl << std::endl;
    return 0;
}

