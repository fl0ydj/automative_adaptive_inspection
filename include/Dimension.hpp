/*
Copyright (c) 2020
TU Berlin, Institut für Werkzeugmaschinen und Fabrikbetrieb
Fachgebiet Industrielle Automatisierungstechnik
Authors: Justin Heinz
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and /or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
DISCLAIMER: THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef DIMENSION_HPP_
#define DIMENSION_HPP_
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PTC;
/**@brief Struct for saving all important information about a tolerance.
@author Justin Heinz
*/
struct Dimension {
        /**
    Tolerance ID as defined in the JT file.
    */
    int id;
    /**
    Clouds of the referenced geometries used to define the tolerance.
    */
    std::vector<PTC::Ptr> references;
    /**
Coefficients of the referenced geometries used to define the tolerance.
*/
    std::vector<double> coeffs;
    /**
    Type of the tolerance: 1 - linear 2 - angular 3 - Radial 4 - curve length (not implemented) 5 - flatness 6 - circularity
    */
    int type;
    /**
    Point to define the position of the 3D text.
    */
    PT textposition;
    /**
    The value of the tolerance.
    */
    double value;
    /**
    The upper allowed deviation.
    */
    double upperDelta;
    /** The lower allowed deviation. */
    double lowerDelta;
    /**
    The measured value for the tolerance.
    */
    double measured;
    /**
    Set to true, if measured value is within specified limits. False otherwise.
    */
    bool ok;
    /**
    Clouds of the referenced geometries used to measure the value for the toleranced.
    */
    std::vector<PTC::Ptr> measuredReferences;
};
#endif