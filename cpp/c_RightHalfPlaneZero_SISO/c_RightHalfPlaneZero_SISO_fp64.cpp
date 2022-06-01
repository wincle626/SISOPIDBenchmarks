// Created by Yun Wu 2022.01.29
// Copyright @ Yun Wu
// UDRC project
// Heriot-Watt University
#include <mex.hpp>
#include "mexAdapter.hpp"
#include "c_RightHalfPlaneZero_SISO.h"
class MexFunction : public matlab::mex::Function {
    matlab::data::ArrayFactory factory;
public:
    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);
        double setpoint = (double) inputs[0][0];
        double dt = (double) inputs[1][0];
        double X1 = (double) inputs[2][0];
        double X2 = (double) inputs[3][0];
        double X3 = (double) inputs[4][0];
        double X4 = (double) inputs[5][0];
        double X5 = (double) inputs[6][0];
        double X6 = (double) inputs[7][0];
        double X7 = (double) inputs[8][0];
        double X8 = (double) inputs[9][0];
        double X = (double) inputs[10][0];
        double E = (double) inputs[11][0];
        double P = (double) inputs[12][0];
        double I = (double) inputs[13][0];
        double D = (double) inputs[14][0];
        double PID = (double) inputs[15][0];
        double Kp = (double) inputs[16][0];
        double Ki = (double) inputs[17][0];
        double Kd = (double) inputs[18][0];
        RightHalfPlaneZero_SISO<double>(setpoint, dt, X1, X2, X3, X4,
             X5, X6, X7, X8, X, E, P, I, D, PID, Kp, Ki, Kd);
        outputs[0] = factory.createScalar(E);
        outputs[1] = factory.createScalar(P);
        outputs[2] = factory.createScalar(I);
        outputs[3] = factory.createScalar(D);
        outputs[4] = factory.createScalar(PID);
        outputs[5] = factory.createScalar(X1);
        outputs[6] = factory.createScalar(X2);
        outputs[7] = factory.createScalar(X3);
        outputs[8] = factory.createScalar(X4);
        outputs[9] = factory.createScalar(X5);
        outputs[10] = factory.createScalar(X6);
        outputs[11] = factory.createScalar(X7);
        outputs[12] = factory.createScalar(X8);
        outputs[13] = factory.createScalar(X);
    }
    void checkArguments(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        matlab::data::ArrayFactory factory;
        if (inputs.size() != 19) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Ten inputs required") }));
        }
        if (inputs[0].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 1 must be a scalar") }));
        }
        
        if (inputs[0].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[0].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 1 must be a noncomplex scalar double") }));
        }
        if (inputs[1].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 2 must be a scalar") }));
        }
        
        if (inputs[1].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[1].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 2 must be a noncomplex scalar double") }));
        }
        if (inputs[2].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 3 must be a scalar") }));
        }
        
        if (inputs[2].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[2].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 3 must be a noncomplex scalar double") }));
        }
        if (inputs[3].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 4 must be a scalar") }));
        }
        
        if (inputs[3].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[3].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 4 must be a noncomplex scalar double") }));
        }
        if (inputs[4].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 5 must be a scalar") }));
        }
        
        if (inputs[4].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[4].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 5 must be a noncomplex scalar double") }));
        }
        if (inputs[5].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 6 must be a scalar") }));
        }
        
        if (inputs[5].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[5].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 6 must be a noncomplex scalar double") }));
        }
        if (inputs[6].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 7 must be a scalar") }));
        }
        
        if (inputs[6].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[6].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 7 must be a noncomplex scalar double") }));
        }
        if (inputs[7].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 8 must be a scalar") }));
        }
        
        if (inputs[7].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[7].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 8 must be a noncomplex scalar double") }));
        }
        if (inputs[8].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 9 must be a scalar") }));
        }
        
        if (inputs[8].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[8].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 9 must be a noncomplex scalar double") }));
        }
        if (inputs[9].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 10 must be a scalar") }));
        }
        
        if (inputs[9].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[9].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 10 must be a noncomplex scalar double") }));
        }
    }
};

