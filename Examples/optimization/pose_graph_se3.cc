/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include <Optimizer.h>


#include<Eigen/StdVector>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include <Converter.h>

#include<mutex>




#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>




//#include <g2o/types/slam3d/types_slam3d.h>


#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"

#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"



#include "System.h"

using namespace std;




int main(int argc, char **argv)
{
    
    if ( argc != 2 )
    {
        cout<<"Usage: pose_graph_g2o_SE3 sphere.g2o"<<endl;
        return 1;
    }
    ifstream fin( argv[1] );
    if ( !fin )
    {
        cout<<"file "<<argv[1]<<" does not exist."<<endl;
        return 1;
    }
    
    
        // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    
    
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    

//     g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    
    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);
    
    
    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();
    
    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的数量
    
    while ( !fin.eof() )
    {
        string name;
        fin>>name;
        if ( name == "VERTEX_SE3:QUAT" )
        {
//             // SE3 顶点
//             g2o::VertexSE3* v = new g2o::VertexSE3();
//             
// 
//             
//             int index = 0;
//             fin>>index;
//             v->setId( index );
//             v->read(fin);
//             
//             optimizer.addVertex(v);
//             vertexCnt++;
//             if ( index==0 )
//                 v->setFixed(true);
            
            
            int index = 0;
            fin>>index;
            
            
            double tx, ty, tz, qx, qy, qz, qw ;
            fin>>tx >> ty>> tz>> qx>> qy>> qz>> qw ; 
            
            Eigen::Quaterniond q ( qw, qx, qy, qz) ; 
            Eigen::Matrix<double,3,3> Rcw = q.toRotationMatrix();
            Eigen::Matrix<double,3,1> tcw;
            tcw << tx , ty, tz ;
            /*
            cout << tcw << endl;*/
            
            
            
            g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();
            
//             Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
//             Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
            
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            
            VSim3->setEstimate(Siw);    
            VSim3->setId(index);
            VSim3->setMarginalized(false);
            VSim3->_fix_scale = true;
            optimizer.addVertex(VSim3);
            vertexCnt++;
            
        }
        
        else if ( name=="EDGE_SE3:QUAT" )
        {
            
            
            
            g2o::EdgeSim3* e = new g2o::EdgeSim3();

            int idx1, idx2;     // 关联的两个顶点
            fin>>idx1>>idx2;            
            
            e->setId( edgeCnt++ );
            
            
            double tx, ty, tz, qx, qy, qz, qw , temp ;
            fin>>tx >> ty>> tz>> qx>> qy>> qz>> qw ; 
            
            fin >>temp >>temp >>temp >>temp >>temp >>temp >>temp   ; 
            fin >>temp >>temp >>temp >>temp >>temp >>temp >>temp   ; 
            fin >>temp >>temp >>temp >>temp >>temp >>temp >>temp   ; 
            
            
            Eigen::Quaterniond q ( qw, qx, qy, qz) ; 
            Eigen::Matrix<double,3,3> Rcw = q.toRotationMatrix();
            Eigen::Matrix<double,3,1> tcw;
            tcw << tx , ty, tz ;


            //10000 0 0 0 0 0 10000    0 0 0 0 10000 0 0         0 40000 0 0 40000 0 40000

            
            
            e->setVertex(1, optimizer.vertices()[idx1] );
            e->setVertex(0, optimizer.vertices()[idx2] );
            
            g2o::Sim3 Sij(Rcw,tcw,1.0); 
            
            e->setMeasurement(Sij);

            e->information() = matLambda;
            optimizer.addEdge(e);
            
            
            
            
//             // SE3-SE3 边
//             g2o::EdgeSE3* e = new g2o::EdgeSE3();
//             int idx1, idx2;     // 关联的两个顶点
//             fin>>idx1>>idx2;
//             e->setId( edgeCnt++ );
//             e->setVertex( 0, optimizer.vertices()[idx1] );
//             e->setVertex( 1, optimizer.vertices()[idx2] );
//             e->read(fin);
//             optimizer.addEdge(e);
//             continue;
            
        }
        if ( !fin.good() ) break;
    }


    
    
    cout<<"read total "<<vertexCnt<<" vertices, "<<edgeCnt<<" edges."<<endl;
    
    
//     /*optimizer*/.save("result.g2o");
    
    
    cout<<"prepare optimizing ..."<<endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    cout<<"calling optimizing ..."<<endl;
    optimizer.optimize(20);
    
    cout<<"saving optimization results ..."<<endl;
    /*
    for ( int i =0 ; i < optimizer.vertices().size() ; i ++ ) {

        
    }*/
//     optimizer.save("result.g2o");
    
    
    
    
    
    
    return 0 ;
    
}
