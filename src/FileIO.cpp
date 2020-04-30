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
#include "FileIO.h"
//HELPER FUNCTION
//Source: SIEMENS PLM (c)
bool processSurface(JtkXTSurface* surface, PTC::Ptr cloud, std::vector<double>& coeffs,PT& textposition)
{
    JtkEntity::TypeID surfaceType = surface->typeID();

    switch (surfaceType)
    {
    case JtkEntity::JtkXTPLANE:
    {
        double* location = NULL;
        double* reference = NULL;
        double* axis = NULL;
        int error = 0;

        ((JtkXTPlane*)surface)->getInternals(location,
            reference,
            axis,
            error);
        coeffs.push_back(*location);
        coeffs.push_back(*(location + 1));
        coeffs.push_back(*(location + 2));
        coeffs.push_back(*axis);
        coeffs.push_back(*(axis + 1));
        coeffs.push_back(*(axis + 2));
        textposition = PT(*location, *(location + 1), *(location + 2));
        JtkEntityFactory::deleteMemory(location);
        location = NULL;

        JtkEntityFactory::deleteMemory(reference);
        reference = NULL;

        JtkEntityFactory::deleteMemory(axis);
        axis = NULL;

        break;
    }
    case JtkEntity::JtkXTCYLINDER:
    {
        double* location = NULL;
        double* reference = NULL;
        double* axis = NULL;
        double radius = 0.0;
        int error = 0;

        ((JtkXTCylinder*)surface)->getInternals(location,
            reference,
            axis,
            radius,
            error);
        coeffs.push_back(*location);
        coeffs.push_back(*(location + 1));
        coeffs.push_back(*(location + 2));
        coeffs.push_back(*axis);
        coeffs.push_back(*(axis + 1));
        coeffs.push_back(*(axis + 2));
        coeffs.push_back(radius);
        JtkEntityFactory::deleteMemory(location);
        location = NULL;

        JtkEntityFactory::deleteMemory(reference);
        reference = NULL;

        JtkEntityFactory::deleteMemory(axis);
        axis = NULL;

        break;
    }
    default:
        return false; //Not implemented: JtkEntity::JtkXTBSURF, JtkEntity::JtkXTMESH, JtkEntity::JtkXTSPUN, JtkEntity::JtkXTSWEPT, JtkEntity::JtkXTOFFSET, JtkEntity::JtkXTTORUS, JtkEntity::JtkXTSPHERE, JtkEntity::JtkXTCONE
        break;
    }
    return true;
}
//HELPER FUNCTION
//Source: SIEMENS PLM (c)
bool processCurve(JtkXTCurve* curve,JtkXTEdge* edge,PTC::Ptr cloud,std::vector<double>& coeffs,PT& textposition, bool limitsSurface)
{
    JtkEntity::TypeID curveType = curve->typeID();

    switch (curveType)
    {
    case JtkEntity::JtkXTLINE:
    {
        //Get Start and End Point
        int error;
        JtkXTSpoke* xtStartSpoke = NULL;
        JtkXTSpoke* xtEndSpoke = NULL;
        JtkXTVertex* startVertex = NULL;
        JtkXTPoint* startPoint = NULL;
        double* startPointXYZ = NULL;
        JtkXTVertex* endVertex = NULL;
        JtkXTPoint* endPoint = NULL;
        double* endPointXYZ = NULL;
        edge->getStartSpoke(xtStartSpoke, error);
        edge->getEndSpoke(xtEndSpoke, error);
        xtStartSpoke->getChild(startVertex, error);
        xtEndSpoke->getChild(endVertex, error);
        startVertex->getPoint(startPoint, error);
        endVertex->getPoint(endPoint, error);
        startPoint->getInternals(startPointXYZ, error);
        endPoint->getInternals(endPointXYZ, error);
        JtkEntityFactory::deleteMemory(startPointXYZ);
        JtkEntityFactory::deleteMemory(endPointXYZ);
        float numOfPoints = 1000.0;
        Eigen::Vector3f start(*(startPointXYZ), *(startPointXYZ + 1), *(startPointXYZ + 2));
        Eigen::Vector3f end(*(endPointXYZ), *(endPointXYZ + 1), *(endPointXYZ + 2));
        for (int i = 0; i < numOfPoints; i++) {
                PT next((end[0]-start[0]) * i / numOfPoints + start[0], (end[1] - start[1]) * i / numOfPoints + start[1], (end[2] - start[2]) * i / numOfPoints + start[2]);
                cloud->push_back(next);
        }
        if (!limitsSurface) {
            PT tmp = PT((end[0]-start[0]) * 0.5 + start[0], (end[1] - start[1]) * 0.5 + start[1], (end[2] - start[2]) * 0.5 + start[2]);
            if (textposition.x==0&& textposition.y == 0&& textposition.z == 0) {
                textposition = tmp;
            }
            else {
                textposition = PT((textposition.x+tmp.x)/2, (textposition.y + tmp.y) / 2, (textposition.z + tmp.z) / 2);
            }
        }
        startPointXYZ = NULL;
        endPointXYZ = NULL;
        break;
    }
    case JtkEntity::JtkXTCIRCLE:
    {
        double* location = NULL;
        double* reference = NULL;
        double* axis = NULL;
        double radius = 0.0;
        int error = 0;

        ((JtkXTCircle*)curve)->getInternals(location,
            reference,
            axis,
            radius,
            error);
        bool radiusEqualsCylRadius = false;
        Eigen::Vector3f loc(*(location), *(location + 1), *(location + 2));
        Eigen::Vector3f _axis(*(axis), *(axis + 1), *(axis + 2));
        Eigen::Vector3f _reference(*(reference), *(reference + 1), *(reference + 2));
        Eigen::Vector3f _reference2 = _axis.cross(_reference);
        if (coeffs.size() > 6) //checks if radius is equal to cyl radius
            if (radius == coeffs.at(6))
                radiusEqualsCylRadius = true;
        if (!limitsSurface || radiusEqualsCylRadius) {
            coeffs.push_back(loc[0]);
            coeffs.push_back(loc[1]);
            coeffs.push_back(loc[2]);
            if (!limitsSurface) {
                coeffs.push_back(radius);
                coeffs.push_back(_axis[0]);
                coeffs.push_back(_axis[1]);
                coeffs.push_back(_axis[2]);
            }
            textposition = PT(loc[0] + radius * _reference2[0], loc[1] + radius * _reference2[1], loc[2] + radius * _reference2[2]);
        }
        int numOfPoints = 1000;
        double t = 0;
        for (int i = 0; i < numOfPoints; i++) {
            t += 2 * M_PI / numOfPoints;
            cloud->push_back(PT(loc[0] + radius * (cos(t) * _reference[0] + sin(t) * _reference2[0]), loc[1] + radius * (cos(t) * _reference[1] + sin(t) * _reference2[1]), loc[2] + radius * (cos(t) * _reference[2] + sin(t) * _reference2[2])));
        }
        JtkEntityFactory::deleteMemory(location);
        location = NULL;

        JtkEntityFactory::deleteMemory(reference);
        reference = NULL;

        JtkEntityFactory::deleteMemory(axis);
        axis = NULL;

        break;
    }
    default:
    {
        return false; //Not Implemented: JtkXTEntity::JtkXTPOLYLINE, JtkEntity::JtkXTSPCURVE, JtkEntity::JtkXTELLIPSE, JtkEntity::JtkXTBCURVE
        break;
    }
    }
    return true;
}
std::vector<Dimension> tmp;
bool findXTBRepwithID(JtkXTBody* body,int id,int type,PTC::Ptr cloud,std::vector<double>& coeffs, PT& textposition) {
	int error;
	int numRegions = body->numChildren(error);
    bool valid = true;
    bool foundSomething=false;
	for (int iRegion = 0; iRegion < numRegions; iRegion++)
	{
		JtkXTRegion* region = NULL;
		body->getChild(region, iRegion, error);
		int numShells = region->numChildren(error);
        for (int iShell = 0; iShell < numShells; iShell++)
        {
            JtkXTShell* shell = NULL;
            region->getChild(shell, iShell, error);
            int numFacades = shell->numFacades(error);
            for (int iFacade = 0; iFacade < numFacades; iFacade++)
            {
                bool surfaceFound = false;
                JtkXTFacade* facade = NULL;
                shell->getChild(facade, iFacade, error);
                JtkXTFace* face = NULL;
                facade->getChild(face, error);
                if (type == 49) { //we are looking for a surface
                    int entityId = 0;
                    face->getEntityId(entityId, error);
                    JtkXTSurface* surface = NULL;
                    face->getSurface(surface, error);
                    if (entityId == id) {
                        if (foundSomething)
                            return valid;
                        surfaceFound=processSurface(surface,cloud,coeffs,textposition);
                        foundSomething = true;
                    }
                }
                int numLoops = face->numChildren(error);
                for (int iLoop = 0; iLoop < numLoops; iLoop++)
                {
                    JtkXTLoop* loop = NULL;
                    face->getChild(loop, iLoop, error);
                    int numFins = loop->numChildren(error);
                    for (int iFin = 0; iFin < numFins; iFin++)
                    {
                        JtkXTFin* fin = NULL;
                        loop->getChild(fin, iFin, error);
                        JtkXTEdge* edge = NULL;
                        fin->getEdge(edge, error);
                        JtkXTCurve* curve = NULL;
                        edge->getCurve(curve, error);
                        int edgeId;
                        edge->getEntityId(edgeId, error);
                        if (edgeId == id) {
                            return processCurve(curve, edge, cloud, coeffs, textposition,false);
                        }
                        else if (type == 49 && surfaceFound)
                            valid=valid&&processCurve(curve, edge, cloud, coeffs, textposition,true);
                    }
                }
            }
        }
	}
    return false;
}
//HELPER FUNCTION
//Source: SIEMENS PLM (c)
int
nodeCrawler(JtkHierarchy* CurrNode, int level, JtkClientData*)
{
	if (CurrNode->isOfSubType(JtkEntity::JtkUNITHIERARCHY))
	{
		int numBodies;
		((JtkPart*)CurrNode)->numXtBodies(numBodies);
		int index = 0;
	    JtkXTBody* part=NULL;
		int error;
		((JtkPart*)CurrNode)->getBody(index, part, error);

		int len_pmi = 0;
		len_pmi = ((JtkUnitHierarchy*)CurrNode)->numPMI();
		for (int index = 0; index < len_pmi; index++)
		{
			JtkPMIEntity* pmi = NULL;
			((JtkUnitHierarchy*)CurrNode)->getPMI(index, pmi);
			if (pmi->isOfSubType(JtkEntity::JtkPMIGENERICENTITY)) //HIER NACH TYPES UNTERSCHEIDEN
			{
				Dimension dim;
				dim.value = 0;
                JtkEntity::TypeID pmiType = pmi->typeID();
                dim.id = pmi->getId();
                switch (pmiType) {
                case JtkEntity::JtkPMIDIMENSION:
                {
                    JtkPMIDimensionType dimType;
                    ((JtkPMIDimension*)(JtkPMIGenericEntity*)pmi)->getType(dimType);
                    dim.type = dimType;
                    ((JtkPMIDimension*)(JtkPMIGenericEntity*)pmi)->getValue(dim.value);
                    ((JtkPMIDimension*)(JtkPMIGenericEntity*)pmi)->getLowerDelta(dim.lowerDelta);
                    ((JtkPMIDimension*)(JtkPMIGenericEntity*)pmi)->getUpperDelta(dim.upperDelta);
                    JtkPMIReference ref;
                    int reason = 0, origin = 0;
                    int ref_len = 0;
                    ((JtkPMIDimension*)(JtkPMIGenericEntity*)pmi)->numReference(ref_len);
                    std::vector<int> ids;
                    for (int i = 0; i < ref_len; i++) {
                        ((JtkPMIDimension*)(JtkPMIGenericEntity*)pmi)->getReference(i, ref, reason, origin);
                        int type = 0;
                        int id = 0;
                        ref.getEntityType(type);
                        ref.getEntityId(id);
                        if (std::find(ids.begin(), ids.end(), id) == ids.end()) {
                            if (type == 48 || type == 49) {
                                PTC::Ptr cloud(new PTC);
                                if (findXTBRepwithID(part, id, type, cloud, dim.coeffs, dim.textposition)) {
                                    dim.references.push_back(cloud);
                                    ids.push_back(id);
                                }
                            }
                        }
                    }
                    tmp.push_back(dim);
                    break;
                }
                case JtkEntity::JtkPMIFEATURECONTROLFRAME:
                {
                    JtkPMIFCFCharacteristic chara;
                    ((JtkPMIFeatureControlFrame*)(JtkPMIGenericEntity*)pmi)->getCharacteristic(chara);
                    if (chara != CharacteristicFLATNESS&& chara != CharacteristicCIRCULARITY)
                        PCL_ERROR("Feature control frame not supported.");
                    else {
                        JtkString value;
                        ((JtkPMIFeatureControlFrame*)(JtkPMIGenericEntity*)pmi)->getToleranceCompartmentValue(int(), value);
                        std::stringstream ss;
                        ss << value;
                        dim.value = stod(ss.str());
                        if (chara == CharacteristicFLATNESS)
                            dim.type = 5;
                        else
                            dim.type = 6;
                        int reason = 0, origin = 0;
                        int ref_len = 0;
                        JtkPMIReference ref;
                        ((JtkPMIFeatureControlFrame*)(JtkPMIGenericEntity*)pmi)->numReference(ref_len);
                        std::vector<int> ids;
                        for (int i = 0; i < ref_len; i++) {
                            ((JtkPMIFeatureControlFrame*)(JtkPMIGenericEntity*)pmi)->getReference(i, ref, reason, origin);
                            int type = 0;
                            int id = 0;
                            ref.getEntityType(type);
                            ref.getEntityId(id);
                            if (std::find(ids.begin(), ids.end(), id) == ids.end()) {
                                if (type == 48 || type == 49) {
                                    PTC::Ptr cloud(new PTC);
                                    dim.textposition = PT(0, 0, 0);
                                    if (findXTBRepwithID(part, id, type, cloud, dim.coeffs,dim.textposition)) {
                                        dim.references.push_back(cloud);
                                        ids.push_back(id);
                                    }
                                }
                            }
                        }
                        tmp.push_back(dim);
                    }
                    break;
                }
                }
			}
		}
        part = NULL;
	}
	return 1;
}
template<typename T>
bool FileIO::loadCloud(std::string text, typename pcl::PointCloud<T>::Ptr cloud, std::string def, bool scalingRequired) {
	std::string path;
	std::cout << text;
	std::cin >> path;
	if (def != "" && path == "default")
		path = def;
    if (path == "q")
        return false;
	cout << "Trying to load " << path << " ...\n";
	try {
		if (path.substr(path.size() - 4) == ".pcd" || path.substr(path.size() - 5) == ".pcd\"")
			pcl::io::loadPCDFile(path, *cloud);
		else if (path.substr(path.size() - 4) == ".stl" || path.substr(path.size() - 5) == ".stl\"") {
			pcl::PolygonMesh mesh;
			pcl::io::loadPolygonFileSTL(path, mesh);
			pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
		}
		else if (path.substr(path.size() - 4) == ".ply" || path.substr(path.size() - 5) == ".ply\"") {
			pcl::PolygonMesh mesh;
            pcl::io::loadPLYFile(path, mesh);
			pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
		}
        else if (path.substr(path.size() - 4) == ".txt" || path.substr(path.size() - 5) == ".txt\"") {
            if (!loadResults(path))
                return false;
            scalingRequired = false;
        }
        else if (path.substr(path.size() - 3) == ".jt" || path.substr(path.size() - 4) == ".jt\"") {
            if (!loadTolerances(path))
                return false;
            scalingRequired = false;
        }
        else {
			PCL_ERROR("File format not supported.");
			return false;
		}
	}
	catch (std::exception e) {
		PCL_ERROR(e.what());
		return false;
	}
	PCL_INFO("File loaded successfully...\n");
	if (scalingRequired) {
		if (!queryScaling<T>(path,cloud))
			return false;
	}
	return true;
}
template bool FileIO::loadCloud<PT>(std::string text, PTC::Ptr cloud, std::string def, bool scalingRequired);
template bool FileIO::loadCloud<pcl::PointXYZRGB>(std::string text, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string def, bool scalingRequired);
bool FileIO::loadTolerances(std::string path) {
    if (path == "") {
        std::cout << "What is the file containing the tolerance information? (supported: .jt)\n";
        std::cin >> path;
    }
    if (path.substr(path.size() - 3) == ".jt" || path.substr(path.size() - 4) == ".jt\"")
    {
        try {
            JtkEntityPtr<JtkCADImporter> cadImp = JtkEntityFactory::createCADImporter();
            // cset the imported to read all the LODs, Tessellation and Brep
            cadImp->setShapeLoadOption(JtkCADImporter::JtkALL_LODS);
            cadImp->setBrepLoadOption(JtkCADImporter::JtkTESS_AND_BREP);
            cadImp->setXTBrepEditOption(JtkCADImporter::JtkXTBREP_FOR_EDIT_ON);
            {
                JtkEntityPtr<JtkHierarchy>  root;
                root = cadImp->import(path.c_str());
                {
                    // Create a traverser to traver down the JtTk graph
                    JtkEntityPtr<JtkTraverser> trav = JtkEntityFactory::createTraverser();
                    trav->setupPreActionCallback(nodeCrawler);
                    trav->traverseGraph(root);
                    trav = NULL;
                }
                root = NULL;
            }
            cadImp = NULL;
        }
        catch (std::exception e) {
            PCL_ERROR("The PMI information could not be extracted.\n");
            return false;
        }
        tolerances = tmp;
        std::cout << "Tolerance definition loaded.\n";
    }
    else {
        PCL_ERROR("Could not read tolerance definition.");
        return false;
    }
	if (!queryScaling<PT>(path))
		return false;
	return true;
}
template<typename T>
bool FileIO::queryScaling(std::string path, typename pcl::PointCloud<T>::Ptr cloud) {
	std::cout << "What unit is used in " << path << "? (mikro,mm,cm,dm,m)\n";
	std::cin >> path;
	double scale;
	if (path == "mikro")scale = 1.0 / 1000000;
	else if (path == "mm") scale = 1.0 / 1000;
	else if (path == "cm") scale = 1.0 / 100;
	else if (path == "dm") scale = 1.0 / 10;
	else if (path == "m") scale = 1;
	else {
		PCL_ERROR("Unit not supported.");
		return false;
	}
	Eigen::MatrixXf transform=Eigen::Matrix4f::Identity();
	transform(0, 0) *= scale;
	transform(1, 1) *= scale;
	transform(2, 2) *= scale;
	if(cloud->width>0)
		pcl::transformPointCloud(*cloud, *cloud, transform);
	else
		for (Dimension& tolerance : tolerances) {
            for (PTC::Ptr points:tolerance.references)
			    pcl::transformPointCloud(*points, *points, transform);
            tolerance.textposition = PT(tolerance.textposition.x* scale, tolerance.textposition.y* scale, tolerance.textposition.z* scale);
            if (tolerance.type != 2) { //for anything except angular dimensions
                tolerance.lowerDelta *= 1.0 / 1000; //tolerance values are given in millimeters
                tolerance.upperDelta *= 1.0 / 1000;
                tolerance.value *= 1.0 / 1000;
            }
            for (double& coeff : tolerance.coeffs)
                coeff *= scale;
		}
	return true;
}
void FileIO::saveParameters(std::map<std::string, std::string>& parameters) {
    std::cout << "Saving parameters to parameters.txt...\n";
    std::ofstream file("parameters.txt", std::ofstream::out | std::ofstream::trunc);
    if (file.is_open()) {
        for (auto const& parameter : parameters)
        {
            file << parameter.first << ':' << parameter.second << "\n";
        }
    }
    file.close();
}
void FileIO::loadParameters(std::map<std::string, std::string>&  parameters) {
    std::cout << "Loading parameters from parameters.txt...\n";
    ifstream file("parameters.txt");
    try {
        if (file.is_open()) {
            std::string key, value;
            while (getline(file, key, ':') && getline(file, value))
                parameters.at(key) = value;
        }
        else
            saveParameters(parameters);
    }
    catch (std::exception e) {
        PCL_ERROR("parameters.txt is corrupt. Delete it manually");
    }
    file.close();
}
template<typename T>
void FileIO::saveCloud(std::string path, typename pcl::PointCloud<T>::Ptr cloud) {
    cout << "Saving point cloud "<< path <<"...\n";
    pcl::io::savePCDFileASCII(path, *cloud);
}
template void FileIO::saveCloud<PT>(std::string name, PTC::Ptr cloud);
template void FileIO::saveCloud<pcl::PointXYZRGB>(std::string name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void FileIO::saveResults() {
    cout << "Saving results in Results.txt...\n";
    std::ofstream file("Results.txt", std::ofstream::out | std::ofstream::trunc);
    if (file.is_open()) {
        for (Dimension tolerance : tolerances)
        {
            file << tolerance.id << ':' <<tolerance.textposition<<':'<< tolerance.value <<':'<<tolerance.measured <<':'<<tolerance.ok<< "\n";
        }
    }
    file.close();
    PTC::Ptr ref(new PTC);
    for (Dimension tolerance : tolerances)
        for (PTC::Ptr reference : tolerance.references)
            ref.reset(new PTC(*ref + *reference));
    saveCloud<PT>("References.pcd", ref);
    ref.reset(new PTC);
    for (Dimension tolerance : tolerances)
        for (PTC::Ptr reference : tolerance.measuredReferences)
            ref.reset(new PTC(*ref + *reference));
    saveCloud<PT>("MeasuredReferences.pcd", ref);
}
bool FileIO::loadResults(std::string path) {
    cout << "Loading results from "<<path<<"...\n";
    try {
        ifstream file(path);
        if (file.is_open()) {
            std::string key, value;
            while (!file.eof())
            {
                Dimension dim;
                std::string id, textposition, value, measured, ok;
                if (getline(file, id, ':') && getline(file, textposition, ':') && getline(file, value, ':') && getline(file, measured, ':')&&getline(file, ok)) {
                    dim.id = stoi(id);
                    std::vector<std::string> position;
                    boost::split(position, textposition, boost::is_any_of(","));
                    dim.textposition = PT(stod(position[0].substr(1)), stod(position[1]), stod(position[2].substr(0, position[2].size() - 1)));
                    dim.value = stod(value);
                    dim.measured = stod(measured);
                    dim.ok = stoi(ok);
                    tolerances.push_back(dim);
                }
                else{
                    getline(file, id);
                    if (id != "") {
                        PCL_ERROR("Cannot parse results file!");
                        return false;
                    }
                }
            }
        }
        file.close();
    }
    catch (std::exception e) {
        PCL_ERROR("Error working with results file..");
        return false;
    }
    cout << "Loading references from References.pcd and MeasuredReferences.pcd...\n";
    PTC::Ptr ref(new PTC);
    PTC::Ptr mref(new PTC);
    try {
            pcl::io::loadPCDFile("References.pcd", *ref);
            tolerances[0].references.push_back(ref);
            pcl::io::loadPCDFile("MeasuredReferences.pcd", *mref);
            tolerances[0].measuredReferences.push_back(mref);
    }
    catch (std::exception e) {
        PCL_ERROR("Cannot load References.pcd or MeasuredReferences.pcd.");
        return false;
    }
    return true;
}
