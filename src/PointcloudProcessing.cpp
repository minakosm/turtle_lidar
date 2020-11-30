#include "PointcloudProcessing.h"
#include "settings.h"
#include "lines.h"
#include "fastcluster.h"
#include "dbscan.h"
#include "utils.h"

#include <eigen3/Eigen/Dense>
#include <limits>
#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>

PointcloudProcessing::PointcloudProcessing() : bF(segments, bins), gF(), cS(), clS(), lines(segments, bins) {
}

PointcloudProcessing::PointcloudProcessing(Eigen::VectorXf *X, Eigen::VectorXf *Y, Eigen::VectorXf *Z) : bF(segments, bins), gF(), cS(), clS(), x(X), y(Y), z(Z), azim(new Eigen::VectorXf(X->rows())), r(new Eigen::VectorXf(X->rows())), lines(segments, bins) {
    polarInit();
}

PointcloudProcessing::PointcloudProcessing(std::unique_ptr<Eigen::VectorXf> &X, std::unique_ptr<Eigen::VectorXf> &Y, std::unique_ptr<Eigen::VectorXf> &Z) : bF(segments, bins), gF(), cS(), clS(), azim(new Eigen::VectorXf(X->rows())), r(new Eigen::VectorXf(X->rows())), lines(segments, bins) {
    this->x.swap(X);
    this->y.swap(Y);
    this->z.swap(Z);
    polarInit();
}

void PointcloudProcessing::polarInit() {
    #pragma omp parallel for num_threads(NUM_OF_THREADS)
        for (int i = 0; i < x->size(); i++) {
            (*azim)(i) = std::atan2((*y)(i), (*x)(i));
        }
    //*azim = y->binaryExpr((*x), [] (float a, float b) {return std::atan2(a,b);});
    (*r).noalias() = (x->array().square() + y->array().square()).sqrt().matrix();
    prototypePointsMatrix = Eigen::MatrixXi::Constant(segments, bins, -1);
    if(bF.filterEnabled == 0) {
        groundArray = Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(partitionMatrix.rows(),1,false);
        partitionMatrix.resize(x->size(), Eigen::NoChange);
    }
}

void PointcloudProcessing::printPointcloudSize() {
    cout << "Pointcloud contains " << x->size() << " points\n";
}

void PointcloudProcessing::printSettings() {
    std::cout << "Base Filter settings:\n" << bF 
         << "Ground Filter settings:\n" << gF
         << "Clustering settings:\n" << cS << std::endl;
}

Eigen::VectorXf PointcloudProcessing::getX() {
    return (*x);
}

Eigen::VectorXf PointcloudProcessing::getY() {
    return (*y);
}

Eigen::VectorXf PointcloudProcessing::getZ() {
    return (*z);
}

Eigen::VectorXf PointcloudProcessing::getAzim() {
    return (*azim);
}

Eigen::VectorXf PointcloudProcessing::getR() {
    return (*r);
}

Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> PointcloudProcessing::getCart() {
    return (*cart);
}

Eigen::VectorXi PointcloudProcessing::getClusters() {
    return clusters;
}

void PointcloudProcessing::filter() {
	removePoints((((azim->array() > bF.maxAzim).cast<int>() + (azim->array() < bF.minAzim).cast<int>())*(int)bF.filterAzim +
                  ((r->array() > bF.maxRad).cast<int>() + (r->array() < bF.minRad).cast<int>())*(int)bF.filterRad +
                  ((z->array() > bF.maxZ).cast<int>() + (z->array() < bF.minZ).cast<int>())*(int)bF.filterZ).cast<bool>()); 
    partitionMatrix.resize(x->size(), Eigen::NoChange);
    groundArray = Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(partitionMatrix.rows(),1,false);
}

void PointcloudProcessing::removePoints(const Eigen::Array <bool, Eigen::Dynamic, 1> &logicalVector) {
	unsigned int size = (logicalVector == 0).count();
	unsigned int counter = 0;
	for(unsigned int i = 0; i < x->rows(); i++) {
		if(logicalVector(i) == 0) {
			(*x)(counter) = (*x)(i);
            (*y)(counter) = (*y)(i);
            (*z)(counter) = (*z)(i);
            (*azim)(counter) = (*azim)(i);
            (*r)(counter++) = (*r)(i);
		}
	}
	x->conservativeResize(size);
    y->conservativeResize(size);
    z->conservativeResize(size);
    azim->conservativeResize(size);
    r->conservativeResize(size);
}

void PointcloudProcessing::calculatePartitionMatrix() {
	float minAzim = azim->minCoeff();
	float maxAzim = azim->maxCoeff();
	float minRad = r->minCoeff();
	float maxRad = r->maxCoeff();
	float shiftAngle = (maxAzim - minAzim) / ((float)segments);
	float shiftRad = (maxRad - minRad) / ((float)bins);
	partitionMatrix.col(0).noalias() = ((azim->array() - minAzim) / shiftAngle).floor().cast<uint8_t>().matrix();
	partitionMatrix.col(1).noalias() = ((r->array() - minRad) / shiftRad).floor().cast<uint8_t>().matrix();
	partitionMatrix.col(0) = (partitionMatrix.col(0).array() > (segments - 1)).select(segments - 1, partitionMatrix.col(0));
	partitionMatrix.col(1) = (partitionMatrix.col(1).array() > (bins - 1)).select(bins - 1, partitionMatrix.col(1));    
	partitionMatrix.col(0) = (partitionMatrix.col(0).array() < 0).select(0, partitionMatrix.col(0));
	partitionMatrix.col(1) = (partitionMatrix.col(1).array() < 0).select(0, partitionMatrix.col(1));    
}

void PointcloudProcessing::calculatePrototypePointsMatrix() {
    Eigen::MatrixXf prototypePointsDistances = Eigen::MatrixXf::Constant(segments, bins, 100000.0f);
	for(int k = 0; k < z->size(); k++) {
		int seg = partitionMatrix(k,0);
		int bin = partitionMatrix(k,1);
		if ((*z)(k) < prototypePointsDistances(seg, bin)) {
			prototypePointsDistances(seg, bin) = (*z)(k);
			prototypePointsMatrix(seg, bin) = k;
		}
	}
    checkPrototypePointsMatrix();
}

void PointcloudProcessing::checkPrototypePointsMatrix() {
    Eigen::Array <bool, Eigen::Dynamic, Eigen::Dynamic> flags = ((prototypePointsMatrix.array() != -1).cast<int>().rowwise().sum().array() == 1).cast<bool>();
    Eigen::Matrix <int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> prototypePointsMatrixCopy = prototypePointsMatrix;
    for(int k = 0; k < segments; k++) {
        if(flags(k)) {
            int nonZeroIndex = 0, pointIndex = 0;
            for(int j=0; j<bins;j++) {
                if(prototypePointsMatrix(k,j) != -1) {
                    nonZeroIndex = j;
                    pointIndex = prototypePointsMatrix(k,j);
                    break;
                }
            }
            float minDist = std::numeric_limits<float>::infinity();
            int closestPointIndex = 0;
            for(int i = 0; i < segments; i++) {
                for(int j = 0; j < bins; j++) {
                    if(prototypePointsMatrix(i,j) == -1 || prototypePointsMatrix(i,j) == pointIndex)
                        continue;
                    int currentIndex = prototypePointsMatrix(i,j);
                    float dist = (*r)(currentIndex)*(*r)(currentIndex)+(*r)(pointIndex)*(*r)(pointIndex)-2*(*r)(currentIndex)*(*r)(pointIndex)*std::cos((*azim)(currentIndex)-(*azim)(pointIndex))+(*z)(currentIndex)*(*z)(currentIndex)+(*z)(pointIndex)*(*z)(pointIndex)-2*(*z)(currentIndex)*(*z)(pointIndex);
                    if(dist < minDist) {
                        minDist = dist;
                        closestPointIndex = currentIndex;
                    }
                }
            }
            prototypePointsMatrixCopy(k, ((nonZeroIndex+1) % bins)) = closestPointIndex;
        }
    }
    prototypePointsMatrix = prototypePointsMatrixCopy;
}

void PointcloudProcessing::groundLinesFit() {
    //#pragma omp parallel for ordered num_threads(NUM_OF_THREADS)
		for (int i = 0; i < prototypePointsMatrix.rows(); i++) {
            SegmentLines line = segmentGroundLinesFit(i);
            //std::cout << line;
            lines.linesMatrix.col(i) = line.linesMatrix;
            lines.linesMedianMatrixX.col(i) = line.linesMedianMatrixX;
            lines.linesMedianMatrixY.col(i) = line.linesMedianMatrixY;
            lines.sizes(i) = line.size;
		}
}

Eigen::VectorXf PointcloudProcessing::getLine(const Eigen::VectorXf &properties) {
	Eigen::VectorXf line(3);
	line(0) = properties(4) / properties(2);
	line(1) = properties(1) - line(0) * properties(0);
	line(2) = sqrtf(1 - (properties(4) / sqrtf(properties(2) * properties(3))) * (properties(4) / sqrtf(properties(2) * properties(3)))) * sqrtf(properties(3));
	return line;
}

Eigen::VectorXf PointcloudProcessing::updateLine(float x, float y, const Eigen::VectorXf &properties) {
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

inline float PointcloudProcessing::distPointFromLine(float x, float y, const Eigen::VectorXf &param) {
	return std::abs(-param(0)*x+y-param(1)) / sqrt(param(0)*param(0)+1);
}

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
                    if ((std::abs(potentialLine(0)) < gF.mMax) && ((potentialLine(0) > gF.mSmall) ||  ((potentialLine(1) < gF.bMax) && (potentialLine(1) > gF.bMin))) && (potentialLine(2) < gF.maxRmse)) {
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
                else if((distPointFromLine((*r)(point),(*z)(point),previousLineParam) < gF.dPrev) || lineVector.size == 0) {
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
	groundArray = ((((-paramM).cwiseProduct((*r))+(*z)-paramB).cwiseAbs()).cwiseQuotient((paramM.cwiseProduct(paramM)+Eigen::VectorXf::Constant(r->rows(),1)).cwiseSqrt())).array() < gF.dGround;
}

void PointcloudProcessing::filterGround() {
    unsigned int size = (groundArray == 0).count();
	unsigned int counter = 0;
    cart = std::make_unique<Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor>>(size,3);
    for(unsigned int i = 0; i < x->rows(); i++) {
		if(groundArray(i) == 0) {
            (*cart)(counter,0) = (*x)(i);
            (*cart)(counter,1) = (*y)(i);
            (*cart)(counter,2) = (*z)(i);
            counter++;
		}
	}
}

void PointcloudProcessing::nonGroundClustering() {
    if(cS.clusteringMethod == 0)
        hierarchicalClustering();
    else if(cS.clusteringMethod == 1)
        DBSCANclustering();
    clustersVectorFun();
}

void PointcloudProcessing::condensedDistances() {
    int N = cart->rows();
	condensedClusterDistances.resize((N * (N-1)) / 2);
	#pragma omp parallel for ordered shared(cart,condensedClusterDistances) num_threads(NUM_OF_THREADS)
        for(int i = 0; i < (N-1); i++)
            (condensedClusterDistances.segment(i*(N-1) - (i*(i-1))/2, N-1-i)).noalias() = (cart->bottomRows(N-i-1).rowwise() - cart->row(i)).array().square().rowwise().sum().matrix().cast<double>();
}

void PointcloudProcessing::hierarchicalClustering() {
    int N = cart->rows();
    int* merge = new int[2*(N-1)];
	double* height = new double[N-1];
	int* labels = new int[N];
    condensedDistances();
	hclust_fast(N, condensedClusterDistances.data(), 0, merge, height);
	cutree_cdist(N, merge, height, (double)cS.hierClusterDist, labels);
    clusters = Eigen::Map<Eigen::VectorXi>(labels, N);
    delete[] merge;
    delete[] height;
    delete[] labels;
}

void PointcloudProcessing::DBSCANclustering() {
	int N = cart->rows();
	std::vector<Point> points(N);
	clusters.resize(N);
	for(int i = 0; i < N; i++) {
		points.at(i).x = (*cart)(i,0);
		points.at(i).y = (*cart)(i,1);
		points.at(i).z = (*cart)(i,2);
		points.at(i).clusterID = -1;
	}
	DBSCAN db(cS.DBminPts, cS.DBepsilon, points);
	db.run();
	for(int i = 0; i < N; i++)
		clusters(i) = db.m_points.at(i).clusterID-1;
}

void PointcloudProcessing::clustersVectorFun() {
    Eigen::VectorXi clustersVector = Eigen::VectorXi::Constant(groundArray.rows(),-1);
    numberOfClusters = clusters.maxCoeff()+1;
    int counter = 0;
    for(int i = 0; i < clustersVector.size(); i++) {
        if(groundArray(i) == true)
            continue;
        else {
            clustersVector(i) = clusters(counter);
            counter++;
        }
    }
    clusters = clustersVector;
}

Eigen::Matrix <float, Eigen::Dynamic, 2, Eigen::RowMajor> PointcloudProcessing::clusterClassifier(GNBC &nbc) {
    Eigen::Matrix <float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> X = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(numberOfClusters, 3);
    Eigen::Matrix <float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> pos = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(numberOfClusters, 2);
    Eigen::Array <bool, Eigen::Dynamic, 1> coneClusters = Eigen::Matrix<bool,Eigen::Dynamic,1>::Constant(numberOfClusters, false).array();
    Eigen::Array <bool, Eigen::Dynamic, 1> ignoreClusters = Eigen::Matrix<bool, Eigen::Dynamic, 1>::Constant(numberOfClusters, false).array();
    //#pragma omp parallel for ordered num_threads(NUM_OF_THREADS) shared(X)
        for(int i = 0; i < numberOfClusters; i++) {
            Eigen::VectorXf xCluster, yCluster, zCluster, xCluster2, yCluster2, zCluster2;
            Eigen::Array <bool, Eigen::Dynamic, 1> mask = (clusters.array() == i);
            
            
            // START OF Cluster check and reconstruct section
            zCluster = sliceVector<float>(*z, mask);
            if(zCluster.size() > clS.ignoreClusterPointsHigh) {
                ignoreClusters(i) = true;
                continue;
            }
            xCluster = sliceVector<float>(*x, mask);
            yCluster = sliceVector<float>(*y, mask);
            float meanZ = zCluster.mean();
            if(clS.reconstructCluster == true) {
                float meanX = xCluster.mean();
                float meanY = yCluster.mean();
                mask = (mask.cast<int>() + ( ((((x->array()-meanX)*(x->array()-meanX) + (y->array()-meanY)*(y->array()-meanY)) < clS.r*clS.r).cast<int>()) * ( (((z->array()-meanZ) > clS.zMin).cast<int>()) * (((z->array()-meanZ) < clS.zMax).cast<int>()) ).cast<int>())).cast<bool>();
                zCluster2 = sliceVector<float>(*z, mask);
                if(zCluster2.size() > clS.ignoreClusterPointsHigh || zCluster2.size() < clS.ignoreClusterPointsLow) {
                    ignoreClusters(i) = true;
                    continue;
                }
                xCluster2 = sliceVector<float>(*x, mask);
                yCluster2 = sliceVector<float>(*y, mask);
            }
            else if(zCluster.size() < clS.ignoreClusterPointsLow) {
                    ignoreClusters(i) = true;
                    continue;
            }
            // END OF Cluster check and reconstruct section
            
            
            Eigen::Vector3f circle;
            if((clS.reconstructCluster == true && clS.useOriginalClusterCircle == true) || (clS.reconstructCluster == false))
                circle = regressCircle(xCluster, yCluster);
            else
                circle = regressCircle(xCluster2, yCluster2);
            if(clS.reconstructCluster == true)
                X.row(i) << circle(2), zCluster2.mean(), zCluster2.size()*(circle(0)*circle(0) + circle(1)*circle(1) + zCluster2.mean()*zCluster2.mean());
            else
                X.row(i) << circle(2), zCluster.mean(), zCluster.size()*(circle(0)*circle(0) + circle(1)*circle(1) + zCluster.mean()*zCluster.mean());
            pos.row(i) << circle(0), circle(1);
        }
    Eigen::VectorXi labels = nbc.labelMatrix(nbc.predictMatrix(X));
    //for(int i = 0; i < X.rows(); i++)
        //std::cout << "X = " << X.row(i) << "\nlabel = " << labels(i) << std::endl << std::endl << std::endl;
    coneClusters = ((ignoreClusters == false).select(labels.array().cast<bool>(), false));
    Eigen::Matrix <float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> pos2 = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero((coneClusters == true).count(), 2);
    int counter = 0;
    for(int i = 0; i < numberOfClusters; i++) {
        if(coneClusters(i) == true)
            pos2.row(counter++) = pos.row(i);
    }
    return pos2;
}

Eigen::Vector3f PointcloudProcessing::regressCircle(const Eigen::VectorXf &xC, const Eigen::VectorXf &yC) {
    Eigen::Vector3f u;
    u << (xC.sum() / xC.size()), (yC.sum() / yC.size()), 0.04;
    float diff = std::numeric_limits<float>::max(); 
    Eigen::MatrixXf J(xC.size(), 3);
    J.col(2) = Eigen::Matrix<float, Eigen::Dynamic, 1>::Constant(xC.size(), -1.0f);
    Eigen::VectorXf f(xC.size());
    for(int i = 0; (i < clS.regressCircleMaxIter && diff > clS.regressCircleDiffThreshold); i++) {
        J.col(0) = (u(0)-xC.array()) / ((u(0)-xC.array()) * (u(0)-xC.array()) + (u(1)-yC.array()) * (u(1)-yC.array())).sqrt();
        J.col(1) = (u(1)-yC.array()) / ((u(0)-xC.array()) * (u(0)-xC.array()) + (u(1)-yC.array()) * (u(1)-yC.array())).sqrt();
        f = ((u(0)-xC.array())*(u(0)-xC.array()) + (u(1)-yC.array()) * (u(1)-yC.array())).sqrt() - u(2);
        Eigen::Vector3f h = J.colPivHouseholderQr().solve(-f);
        diff = std::sqrt((h.array()*h.array()).sum());
        u += h;
    }
    return u;
}

Eigen::Matrix <float, Eigen::Dynamic, 2, Eigen::RowMajor> PointcloudProcessing::pipeline(std::unique_ptr<Eigen::VectorXf> &X, std::unique_ptr<Eigen::VectorXf> &Y, std::unique_ptr<Eigen::VectorXf> &Z, GNBC &nbc) {
    this->x.swap(X);
    this->y.swap(Y);
    this->z.swap(Z);
    azim = std::make_unique<Eigen::VectorXf>(x->size());
    r = std::make_unique<Eigen::VectorXf>(x->size());
    polarInit();
    filter();
    calculatePartitionMatrix();
    calculatePrototypePointsMatrix();
    checkPrototypePointsMatrix();
    groundLinesFit();
    groundClassifier();
    filterGround();
    nonGroundClustering();
    Eigen::Matrix <float, Eigen::Dynamic, 2, Eigen::RowMajor> conePos = clusterClassifier(nbc);
    return conePos;
}
