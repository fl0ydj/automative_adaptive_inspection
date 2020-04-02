#include "FileIO.h"
bool processSurface(JtkXTSurface* surface, PTC::Ptr cloud, std::vector<double>& coeffs)
{
    JtkEntity::TypeID surfaceType = surface->typeID();

    switch (surfaceType)
    {
    case JtkEntity::JtkXTBSURF:
    {
        return false; //NOT IMPLEMENTED
       /* int uDegree = 0;
        int vDegree = 0;
        int uNumCtlPts = 0;
        int vNumCtlPts = 0;
        int dimension = 0;
        bool isRational = false;
        double* ctlPts = NULL;
        int uNumKnots = 0;
        int vNumKnots = 0;
        int* uKnotMult = NULL;
        int* vKnotMult = NULL;
        double* uKnots = NULL;
        double* vKnots = NULL;
        bool uIsPeriodic = false;
        bool vIsPeriodic = false;
        bool uIsClosed = false;
        bool vIsClosed = false;
        int error = 0;

        ((JtkXTBSurf*)surface)->getInternals(uDegree,
            vDegree,
            uNumCtlPts,
            vNumCtlPts,
            dimension,
            isRational,
            ctlPts,
            uNumKnots,
            vNumKnots,
            uKnotMult,
            vKnotMult,
            uKnots,
            vKnots,
            uIsPeriodic,
            vIsPeriodic,
            uIsClosed,
            vIsClosed,
            error);

        JtkEntityFactory::deleteMemory(ctlPts);
        ctlPts = NULL;

        JtkEntityFactory::deleteMemory(uKnotMult);
        uKnotMult = NULL;

        JtkEntityFactory::deleteMemory(vKnotMult);
        vKnotMult = NULL;

        JtkEntityFactory::deleteMemory(uKnots);
        uKnots = NULL;

        JtkEntityFactory::deleteMemory(vKnots);
        vKnots = NULL;

        break;*/
    }
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

        JtkEntityFactory::deleteMemory(location);
        location = NULL;

        JtkEntityFactory::deleteMemory(reference);
        reference = NULL;

        JtkEntityFactory::deleteMemory(axis);
        axis = NULL;

        break;
    }
    case JtkEntity::JtkXTCONE:
    {
        double* location = NULL;
        double* reference = NULL;
        double* axis = NULL;
        double radius = 0.0;
        double halfAngle = 0.0;
        int error = 0;

        ((JtkXTCone*)surface)->getInternals(location,
            reference,
            axis,
            radius,
            halfAngle,
            error);

        JtkEntityFactory::deleteMemory(location);
        location = NULL;

        JtkEntityFactory::deleteMemory(reference);
        reference = NULL;

        JtkEntityFactory::deleteMemory(axis);
        axis = NULL;

        break;
    }
    case JtkEntity::JtkXTSPHERE:
    {
        double* location = NULL;
        double* reference = NULL;
        double* axis = NULL;
        double radius = 0.0;
        int error = 0;

        ((JtkXTSphere*)surface)->getInternals(location,
            reference,
            axis,
            radius,
            error);

        JtkEntityFactory::deleteMemory(location);
        location = NULL;

        JtkEntityFactory::deleteMemory(reference);
        reference = NULL;

        JtkEntityFactory::deleteMemory(axis);
        axis = NULL;

        break;
    }
    case JtkEntity::JtkXTTORUS:
    {
        double* location = NULL;
        double* reference = NULL;
        double* axis = NULL;
        double majorRadius = 0.0;
        double minorRadius = 0.0;
        int error = 0;

        ((JtkXTTorus*)surface)->getInternals(location,
            reference,
            axis,
            majorRadius,
            minorRadius,
            error);

        JtkEntityFactory::deleteMemory(location);
        location = NULL;

        JtkEntityFactory::deleteMemory(reference);
        reference = NULL;

        JtkEntityFactory::deleteMemory(axis);
        axis = NULL;

        break;
    }
    case JtkEntity::JtkXTOFFSET:
    {
        JtkXTSurface* underlyingSurface = NULL;
        double offsetDistance = 0.0;
        int error = 0;

        ((JtkXTOffset*)surface)->getInternals(underlyingSurface,
            offsetDistance,
            error);
        break;
    }
    case JtkEntity::JtkXTSWEPT:
    {
        JtkXTCurve* curve = NULL;
        double* direction = NULL;
        int error = 0;

        ((JtkXTSwept*)surface)->getInternals(curve,
            direction,
            error);

        JtkEntityFactory::deleteMemory(direction);
        direction = NULL;

        break;
    }
    case JtkEntity::JtkXTSPUN:
    {
        JtkXTCurve* curve = NULL;
        double* location = NULL;
        double* axis = NULL;
        int error = 0;

        ((JtkXTSpun*)surface)->getInternals(curve,
            location,
            axis,
            error);

        JtkEntityFactory::deleteMemory(location);
        location = NULL;

        JtkEntityFactory::deleteMemory(axis);
        axis = NULL;

        break;
    }
    case JtkEntity::JtkXTMESH:
    {
        // Get the internals of the mesh
        int error = 0;

        // 2 posible methods:
        //   All facets of the mesh at once
        //   Get facets in batches in an iterative process

        JtkEntityPtr<JtkXTMesh> mesh = (JtkXTMesh*)surface;

        // Get the total number of facets in the mesh. This could be
        // be used to determine the batch size of the number of facets
        // to return in an iterative process.
        int numFacets = 0;
        mesh->getNumberOfFacets(numFacets, error);

        // Get all the facets at once
        int* facetIds = NULL;      // Each facet of a mesh has a given (unique identifier) (size: numFacets)

        int* vertexIds = NULL;     // Each vertex of the mesh has a given identifier. Vertices that are common
                                   // between facets will share the same unique identifer (size: numFacets * 3)

        double* positions = NULL;  // Coordinates for the facets. This will be provided in the following form:
                                   // facet1(x,y,z), facet2(x,y,z), ..., facetn(x,y,z)

        // Passing in NULL for the 'normals' argument to not request the return of normal information.
        mesh->getInternals(numFacets, facetIds, vertexIds, positions, NULL, error);

        JtkEntityFactory::deleteMemory(facetIds);
        facetIds = NULL;

        JtkEntityFactory::deleteMemory(vertexIds);
        vertexIds = NULL;

        JtkEntityFactory::deleteMemory(positions);
        positions = NULL;

        // Get facets in batches
        int batchSize = 10;
        numFacets = 0;

        // The id for the facet for the start of the getting the facets should be set to 0
        int startingId = 0;
        while (startingId != -1 && mesh->getInternals(startingId, batchSize, numFacets, facetIds, vertexIds, positions, NULL, error) == Jtk_OK && error == 0)
        {
            // Work out the last facetId that we have, for any subsequent iteration
            if (numFacets > 0 && numFacets == batchSize)
            {
                startingId = facetIds[numFacets - 1];
            }
            else
            {
                startingId = -1;
            }

            // free memory
            JtkEntityFactory::deleteMemory(facetIds);
            facetIds = NULL;

            JtkEntityFactory::deleteMemory(vertexIds);
            vertexIds = NULL;

            JtkEntityFactory::deleteMemory(positions);
            positions = NULL;
        }

        break;
    }

    default:
        break;
    }
    return true;
}
void processCurve(JtkXTCurve* curve, JtkXTSurface* surface,JtkXTEdge* edge,PTC::Ptr cloud,std::vector<double>& coeffs)
{
    JtkEntity::TypeID curveType = curve->typeID();

    switch (curveType)
    {
    case JtkEntity::JtkXTBCURVE:
    {
          //NOTIMPLEMENTED
    //    int degree = 0;
    //    int numCtlPts = 0;
    //    int dimension = 0;
    //    bool isRational = false;
    //    double* ctlPts = NULL;
    //    int numKnots = 0;
    //    int* knotMult = NULL;
    //    double* knots = NULL;
    //    bool isPeriodic = false;
    //    bool isClosed = false;
    //    int error = 0;

    //    ((JtkXTBCurve*)curve)->getInternals(degree,
    //        numCtlPts,
    //        dimension,
    //        isRational,
    //        ctlPts,
    //        numKnots,
    //        knotMult,
    //        knots,
    //        isPeriodic,
    //        isClosed,
    //        error);

    //    JtkEntityFactory::deleteMemory(ctlPts);
    //    ctlPts = NULL;

    //    JtkEntityFactory::deleteMemory(knotMult);
    //    knotMult = NULL;

    //    JtkEntityFactory::deleteMemory(knots);
    //    knots = NULL;

        break;
    }
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
        for (int i = 0; i < numOfPoints; i++) {
            PT next((*(endPointXYZ)-*(startPointXYZ)) * i / numOfPoints + *(startPointXYZ), (*(endPointXYZ + 1) - *(startPointXYZ + 1)) * i / numOfPoints + *(startPointXYZ + 1), (*(endPointXYZ + 2) - *(startPointXYZ + 2)) * i / numOfPoints + *(startPointXYZ + 2));
            cloud->push_back(next);
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
        coeffs.push_back(*location);
        coeffs.push_back(*(location+1));
        coeffs.push_back(*(location + 2));
        coeffs.push_back(radius);
        coeffs.push_back(*axis);
        coeffs.push_back(*(axis + 1));
        coeffs.push_back(*(axis + 2));
        Eigen::Vector3f _axis(*(axis), *(axis + 1), *(axis + 2));
        Eigen::Vector3f _reference(*(reference), *(reference + 1), *(reference + 2));
        Eigen::Vector3f _reference2 = _axis.cross(_reference);
        int numOfPoints = 1000;
        double t = 0;
        for (int i = 0; i < numOfPoints; i++) {
            t += 2 * M_PI / numOfPoints;
            cloud->push_back(PT(*(location)+radius * (cos(t)*_reference[0]+sin(t)* _reference2[0]), *(location+1)+radius * (cos(t) * _reference[1] + sin(t) * _reference2[1]), *(location+2)+radius * (cos(t) * _reference[2] + sin(t) * _reference2[2])));
        }
        JtkEntityFactory::deleteMemory(location);
        location = NULL;

        JtkEntityFactory::deleteMemory(reference);
        reference = NULL;

        JtkEntityFactory::deleteMemory(axis);
        axis = NULL;

        break;
    }
    case JtkEntity::JtkXTELLIPSE:
    {
       /* double* location = NULL;
        double* reference = NULL;
        double* axis = NULL;
        double radius1 = 0.0;
        double radius2 = 0.0;
        int error = 0;

        ((JtkXTEllipse*)curve)->getInternals(location,
            reference,
            axis,
            radius1,
            radius2,
            error);

        JtkEntityFactory::deleteMemory(location);
        location = NULL;

        JtkEntityFactory::deleteMemory(reference);
        reference = NULL;

        JtkEntityFactory::deleteMemory(axis);
        axis = NULL;*/

        break;
    }
    case JtkEntity::JtkXTSPCURVE:
    {
       /* JtkXTSurface* surface = NULL;
        JtkXTCurve* curve2 = NULL;
        int error = 0;

        ((JtkXTSPCurve*)curve)->getInternals(surface,
            curve2,
            error);*/

        break;
    }
    case JtkXTEntity::JtkXTPOLYLINE:
    {/*
        int error = 0;

        int numSegments = 0;
        double* segments = NULL;
        int closed = 0;
        double baseParameter = 0.0;
        ((JtkXTPolyline*)curve)->getInternals(numSegments, segments, closed, baseParameter, error);

        JtkEntityFactory::deleteMemory(segments);
        segments = NULL;*/

        break;
    }
    default:
    {
        break;
    }
    }
}
std::vector<Dimension> tmp;
bool findXTBRepwithID(JtkXTBody* body,int id,int type,PTC::Ptr cloud,std::vector<double>& coeffs) {
	int error;
	int numRegions = body->numChildren(error);
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
                        //processSurface(surface,PTC::Ptr cloud,std::vector<double>& coeffs);
                        return false;
                    }
                }
                else if (type == 48) {
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
                                JtkXTSurface* surface = NULL;
                                face->getSurface(surface, error);
                                processCurve(curve,surface,edge,cloud,coeffs);
                                return true;
                            }
                        }
                    }
                }
            }
            //HIER JETZT EDGES OHNE FACES? GEHT DAS
        }
	}
    return false;
}
//COPYRIGHT SIEMENS
int
nodeCrawler(JtkHierarchy* CurrNode, int level, JtkClientData*)
{
	if (CurrNode->isOfSubType(JtkEntity::JtkUNITHIERARCHY))
	{
		int numBodies;
		((JtkPart*)CurrNode)->numXtBodies(numBodies);
		int index = 0;
		JtkXTBody* part;
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
                switch (pmiType){
                case JtkEntity::JtkPMIDIMENSION:
                    {
                        JtkPMIDimensionType dimType;
                        ((JtkPMIDimension*)(JtkPMIGenericEntity*)pmi)->getType(dimType);
                        dim.type = dimType;
                        cout << dim.type;
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
                                    if (findXTBRepwithID(part, id, type, cloud,dim.coeffs)) {
                                        dim.references.push_back(cloud);
                                        ids.push_back(id);
                                    }

                                }
                            }
                        }
                        break;
                    }
                }
                tmp.push_back(dim);
			}
		}
	}
	return 1;
}

bool FileIO::loadCloud(std::string text, PTC::Ptr cloud, std::string def, bool scalingRequired) {
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
			pcl::io::loadPolygonFilePLY(path, mesh);
			pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
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
		if (!queryScaling(path,cloud))
			return false;
	}
	return true;
}
bool FileIO::loadTolerances() {
	std::string path;
	std::cout << "What is the file containing the tolerance information? (supported: .jt; type \"no\" if tolerances are not needed)\n";
	std::cin >> path;
	if (path != "no" || (path.substr(path.size() - 4) != ".jt" && path.substr(path.size() - 5) != ".jt\""))
	{
		JtkEntityFactory::init(JtkEntityFactory::JtkPARASOLID_ON, JtkEntityFactory::JtkENVIRONMENT_CURRENT);
		try {
			JtkEntityPtr<JtkCADImporter> cadImp = JtkEntityFactory::createCADImporter();
			// cset the imported to read all the LODs, Tessellation and Brep
			cadImp->setShapeLoadOption(JtkCADImporter::JtkALL_LODS);
			cadImp->setBrepLoadOption(JtkCADImporter::JtkTESS_AND_BREP);
			cadImp->setXTBrepEditOption(JtkCADImporter::JtkXTBREP_FOR_EDIT_ON);
			JtkEntityPtr<JtkHierarchy>  root;
			root = cadImp->import(path.c_str());
			// Create a traverser to traver down the JtTk graph
			JtkEntityPtr<JtkTraverser> trav = JtkEntityFactory::createTraverser();
            trav->setupPreActionCallback(nodeCrawler);
			//trav->setupPostActionCallback(postAction);
			trav->traverseGraph(root);
		}
		catch (std::exception e) {
			PCL_ERROR("The PMI information could not be extracted.\n");
			return false;
		}
		JtkEntityFactory::fini();
		tolerances = tmp;
		std::cout << "Tolerance definition loaded.\n";
	}
	if (!queryScaling(path))
		return false;
	return true;
}
bool FileIO::queryScaling(std::string path, PTC::Ptr cloud) {
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
            tolerance.lowerDelta *= 1.0/1000; //tolerance values are given in millimeters
            tolerance.upperDelta *= 1.0/1000;
            tolerance.value *= 1.0/1000;
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
    if (file.is_open()) {
        std::string key, value;
        while (getline(file, key, ':') && getline(file, value))
            parameters.at(key) = value;
    }
    else
        saveParameters(parameters);
    file.close();
}