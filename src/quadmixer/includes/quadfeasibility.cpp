#include "quadfeasibility.h"

#include <vcg/complex/algorithms/polygon_polychord_collapse.h>
#include <vcg/complex/algorithms/polygonal_algorithms.h>

#include <quadretopology/includes/qr_utils.h>

namespace QuadBoolean {
namespace internal {

template <class MeshType>
struct PolyChordData {
    size_t startComponent;
    size_t endComponent;

    size_t length;

    std::vector<size_t> quads;
    std::vector<size_t> edges;

    std::pair<typename MeshType::CoordType, typename MeshType::CoordType> firstEdge;
    std::pair<typename MeshType::CoordType, typename MeshType::CoordType> lastEdge;
};

template <class PolyMeshType, class TriangleMeshType>
size_t solveSplittingPolychords(
        PolyMeshType& preservedSurface,
        TriangleMeshType& newSurface,
        std::vector<std::set<std::pair<typename TriangleMeshType::CoordType, typename TriangleMeshType::CoordType>>>& newSurfaceBorderEdges,
        std::vector<bool>& isComponentFeasible);

template <class PolyMeshType, class TriangleMeshType>
size_t solveQuadDominant(
        PolyMeshType& preservedSurface,
        TriangleMeshType& newSurface,
        std::vector<std::set<std::pair<typename TriangleMeshType::CoordType, typename TriangleMeshType::CoordType>>>& newSurfaceBorderEdges,
        std::vector<bool>& isComponentFeasible);

template<class MeshType>
void splitEdgesMappedPoint(
        MeshType& newSurface,
        const std::map<std::pair<typename MeshType::CoordType, typename MeshType::CoordType>, typename MeshType::CoordType>& splittedMap);

template<class MeshType>
std::vector<std::set<std::pair<typename MeshType::CoordType, typename MeshType::CoordType>>> findComponentsBorderEdges(
        MeshType& mesh,
        const std::vector<std::vector<size_t>>& components);

template<class MeshType>
std::vector<bool> findComponentsFeasibility(
        const std::vector<std::set<std::pair<typename MeshType::CoordType,typename MeshType::CoordType>>>& borderEdges);

size_t countNonFeasible(
        const std::vector<bool>& isComponentFeasible);

template<class MeshType>
bool checkFeasibility(
        MeshType& mesh);


template <class PolyMeshType, class TriangleMeshType>
bool checkBorderConsistency(
        PolyMeshType& preservedSurface,
        TriangleMeshType& newSurface);



template <class PolyMeshType, class TriangleMeshType>
FeasibilityResult solveFeasibility(
        PolyMeshType& preservedSurface,
        TriangleMeshType& newSurface,
        const bool polychordSolver,
        const bool splitSolver)
{
    //Update attributes
    vcg::tri::UpdateTopology<TriangleMeshType>::FaceFace(newSurface);
    vcg::tri::UpdateFlags<TriangleMeshType>::FaceBorderFromFF(newSurface);
    vcg::tri::UpdateFlags<TriangleMeshType>::VertexBorderFromFaceAdj(newSurface);
    vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(preservedSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::FaceBorderFromFF(preservedSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::VertexBorderFromFaceAdj(preservedSurface);

    if (!checkBorderConsistency(preservedSurface, newSurface)) {
        return NonConsistant;
    }

    //Find connected components
    std::vector<std::vector<size_t>> components =
            QuadRetopology::internal::findConnectedComponents<TriangleMeshType>(newSurface);

    //Find border vertices
    std::vector<std::set<std::pair<typename TriangleMeshType::CoordType, typename TriangleMeshType::CoordType>>> newSurfaceBorderEdges =
            findComponentsBorderEdges(newSurface, components);

    //Find feasible components
    std::vector<bool> isComponentFeasible =
            findComponentsFeasibility<TriangleMeshType>(newSurfaceBorderEdges);
    size_t numNonFeasible = countNonFeasible(isComponentFeasible);


    //All feasible
    if (numNonFeasible == 0) {
#ifndef NDEBUG
        assert(checkFeasibility(newSurface));
#endif
        return AlreadyOk;
    }

#ifndef NDEBUG
    assert(!checkFeasibility(newSurface));
#endif

    //Only quads
    if (polychordSolver) {
        //Split polychords to solve the feasibility
        numNonFeasible = solveSplittingPolychords(
                    preservedSurface,
                    newSurface,
                    newSurfaceBorderEdges,
                    isComponentFeasible);

        //Update attributes
        vcg::tri::UpdateTopology<TriangleMeshType>::FaceFace(newSurface);
        vcg::tri::UpdateFlags<TriangleMeshType>::FaceBorderFromFF(newSurface);
        vcg::tri::UpdateFlags<TriangleMeshType>::VertexBorderFromFaceAdj(newSurface);
        vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(preservedSurface);
        vcg::tri::UpdateFlags<PolyMeshType>::FaceBorderFromFF(preservedSurface);
        vcg::tri::UpdateFlags<PolyMeshType>::VertexBorderFromFaceAdj(preservedSurface);


        if (numNonFeasible == 0) {
#ifndef NDEBUG
            assert(checkFeasibility(newSurface));
#endif
            return SolvedQuadOnly;
        }
    }

    if (splitSolver) {
        //Quad dominant solution
        numNonFeasible = solveQuadDominant(
                    preservedSurface,
                    newSurface,
                    newSurfaceBorderEdges,
                    isComponentFeasible);

        //Update attributes
        vcg::tri::UpdateTopology<TriangleMeshType>::FaceFace(newSurface);
        vcg::tri::UpdateFlags<TriangleMeshType>::FaceBorderFromFF(newSurface);
        vcg::tri::UpdateFlags<TriangleMeshType>::VertexBorderFromFaceAdj(newSurface);
        vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(preservedSurface);
        vcg::tri::UpdateFlags<PolyMeshType>::FaceBorderFromFF(preservedSurface);
        vcg::tri::UpdateFlags<PolyMeshType>::VertexBorderFromFaceAdj(preservedSurface);
    }

    if (numNonFeasible == 0) {
#ifndef NDEBUG
        assert(checkFeasibility(newSurface));
#endif
        return SolvedQuadDominant;
    }

    return NonSolved;
}


template <class PolyMeshType, class TriangleMeshType>
size_t solveSplittingPolychords(
        PolyMeshType& preservedSurface,
        TriangleMeshType& newSurface,
        std::vector<std::set<std::pair<typename TriangleMeshType::CoordType, typename TriangleMeshType::CoordType>>>& newSurfaceBorderEdges,
        std::vector<bool>& isComponentFeasible)
{
    size_t numNonFeasible = countNonFeasible(isComponentFeasible);
    assert(numNonFeasible > 0);

    //Map for associating edges to component
    std::map<std::pair<typename TriangleMeshType::CoordType, typename TriangleMeshType::CoordType>, size_t> borderComponentsMap;
    for (size_t i = 0; i < newSurfaceBorderEdges.size(); i++) {
        for (const std::pair<typename TriangleMeshType::CoordType, typename TriangleMeshType::CoordType>& edge : newSurfaceBorderEdges[i]) {
            borderComponentsMap.insert(std::make_pair(edge, i));
        }
    }

    //Find candidates to be splitted
    std::vector<PolyChordData<PolyMeshType>> candidatePolychords;
    for (size_t i = 0; i < preservedSurface.face.size(); i++) {
        for (size_t j = 0; j < preservedSurface.face[i].VN(); j++) {
            if (vcg::face::IsBorder(preservedSurface.face[i], j)) {
                typename PolyMeshType::CoordType firstV;
                typename PolyMeshType::CoordType secondV;

                PolyChordData<PolyMeshType> polychord;
                polychord.length = 0;

                //Get first edge
                firstV = preservedSurface.face[i].V0(j)->P();
                secondV = preservedSurface.face[i].V1(j)->P();
                if (firstV < secondV)
                    std::swap(firstV, secondV);

                //First edge and start component
                polychord.firstEdge = std::make_pair(firstV, secondV);
                polychord.startComponent = borderComponentsMap.at(polychord.firstEdge);

                //Go out if the component is feasible
                if (isComponentFeasible.at(polychord.startComponent))
                    continue;

                //Iterator variables
                size_t currentFace = i;
                size_t currentEdge = j;

                //Polygonal face flag
                bool isPolygonal = false;
                do {
                    //Check if quad
                    if (preservedSurface.face[currentFace].VN() == 4) {
                        //Update polychord data
                        polychord.quads.push_back(currentFace);
                        polychord.edges.push_back(currentEdge);
                        polychord.length++;

                        //Next
                        size_t frontEdge = (currentEdge + 2) % 4;
                        currentEdge = preservedSurface.face[currentFace].FFi(frontEdge);
                        currentFace = vcg::tri::Index(preservedSurface, preservedSurface.face[currentFace].FFp(frontEdge));
                    }
                    //Non quad
                    else {
                        isPolygonal = true;
                    }
                } while (!isPolygonal && !vcg::face::IsBorder(preservedSurface.face[currentFace], static_cast<int>(currentEdge)));

                //Go out if it is non quad
                if (isPolygonal)
                    continue;

                //Find last edge
                firstV = preservedSurface.face[currentFace].V0(currentEdge)->P();
                secondV = preservedSurface.face[currentFace].V1(currentEdge)->P();
                if (firstV < secondV)
                    std::swap(firstV, secondV);

                //Last edge and end component
                polychord.lastEdge = std::make_pair(firstV, secondV);
                polychord.endComponent = borderComponentsMap.at(polychord.lastEdge);

                //Go out if the component is feasible
                if (isComponentFeasible.at(polychord.endComponent))
                    continue;

                //The polychord must connect different components
                if (polychord.startComponent == polychord.endComponent)
                    continue;

                candidatePolychords.push_back(polychord);
            }
        }
    }

    //Sort by length
    std::sort(candidatePolychords.begin(), candidatePolychords.end(),
        [](const PolyChordData<PolyMeshType>& a, const PolyChordData<PolyMeshType>& b) -> bool
        {
            return a.length < b.length;
        }
    );

    //Split polychords
    std::map<std::pair<typename PolyMeshType::CoordType, typename PolyMeshType::CoordType>, typename PolyMeshType::CoordType> splittedMap;
    size_t currentPolychord = 0;
    while (numNonFeasible > 1 && currentPolychord < candidatePolychords.size()) {
        PolyChordData<PolyMeshType>& polychord = candidatePolychords[currentPolychord];
		
        if (!isComponentFeasible[polychord.startComponent] && !isComponentFeasible[polychord.endComponent]) {
			size_t startFaceId = polychord.quads[0];
			size_t startEdgeId = polychord.edges[0];

			std::pair<typename PolyMeshType::CoordType, typename PolyMeshType::CoordType> firstEdge = polychord.firstEdge;
			std::pair<typename PolyMeshType::CoordType, typename PolyMeshType::CoordType> lastEdge = polychord.lastEdge;

			vcg::face::Pos<typename PolyMeshType::FaceType> pos(&preservedSurface.face[startFaceId], startEdgeId);

            size_t newFirstPoint = preservedSurface.vert.size();
			vcg::tri::PolychordCollapse<PolyMeshType>::SplitPolychord(preservedSurface, pos, 2);
            size_t newLastPoint = preservedSurface.vert.size()-1;

			splittedMap.insert(std::make_pair(firstEdge, preservedSurface.vert[newFirstPoint].P()));
			splittedMap.insert(std::make_pair(lastEdge, preservedSurface.vert[newLastPoint].P()));

			//Update non feasible
            numNonFeasible -= 2;

            std::cout << "Feasibility solver: polychord splitted." << std::endl;

			isComponentFeasible[polychord.startComponent] = true;
			isComponentFeasible[polychord.endComponent] = true;
		}

        currentPolychord++;
    }

    //Split mapped edges
    splitEdgesMappedPoint(newSurface, splittedMap);

    return numNonFeasible;
}



template <class PolyMeshType, class TriangleMeshType>
size_t solveQuadDominant(
        PolyMeshType& preservedSurface,
        TriangleMeshType& newSurface,
        std::vector<std::set<std::pair<typename TriangleMeshType::CoordType, typename TriangleMeshType::CoordType>>>& newSurfaceBorderEdges,
        std::vector<bool>& isComponentFeasible)
{
    size_t numNonFeasible = countNonFeasible(isComponentFeasible);
    assert(numNonFeasible > 0);

    //Map for associating edges to component
    std::map<std::pair<typename TriangleMeshType::CoordType, typename TriangleMeshType::CoordType>, size_t> borderComponentsMap;
    for (size_t i = 0; i < newSurfaceBorderEdges.size(); i++) {
        for (const std::pair<typename TriangleMeshType::CoordType, typename TriangleMeshType::CoordType>& edge : newSurfaceBorderEdges[i]) {
            borderComponentsMap.insert(std::make_pair(edge, i));
        }
    }

    //Edges to be split in the new surface
    std::map<std::pair<typename PolyMeshType::CoordType, typename PolyMeshType::CoordType>, typename PolyMeshType::CoordType> splittedMap;

    do {
        //Find the polygon to be split
        size_t bestFace = std::numeric_limits<size_t>::max();
        size_t bestEdge = std::numeric_limits<size_t>::max();
        size_t bestComponent = std::numeric_limits<size_t>::max();

        vcg::PolygonalAlgorithm<PolyMeshType>::UpdateQuality(preservedSurface, vcg::PolygonalAlgorithm<PolyMeshType>::QAngle);

        for (size_t i = 0; i < preservedSurface.face.size(); i++) {
            if (!preservedSurface.face[i].IsD() && (
                    bestFace == std::numeric_limits<size_t>::max() ||
                    preservedSurface.face[i].Q()/preservedSurface.face[i].VN() >= preservedSurface.face[bestFace].Q()/preservedSurface.face[bestFace].VN()
            )) {
                for (size_t j = 0; j < preservedSurface.face[i].VN(); j++) {

                    if (!vcg::face::IsBorder(preservedSurface.face[i], j))
                        continue;

                    //Get edge
                    typename PolyMeshType::CoordType firstV = preservedSurface.face[i].V0(j)->P();
                    typename PolyMeshType::CoordType secondV = preservedSurface.face[i].V1(j)->P();
                    if (firstV < secondV)
                        std::swap(firstV, secondV);

                    typename std::map<std::pair<typename TriangleMeshType::CoordType, typename TriangleMeshType::CoordType>, size_t>::iterator it =
                        borderComponentsMap.find(std::make_pair(firstV, secondV));

                    //It can happen only if that component has been already splitted (it is already feasible)
                    if (it == borderComponentsMap.end())
                        continue;

                    size_t cId = it->second;

                    //If the component is not feasible
                    if (!isComponentFeasible.at(cId)) {
                        if (bestFace == i) {
                            double edgeLength = (preservedSurface.face[i].V0(j)->P() - preservedSurface.face[i].V1(j)->P()).Norm();
                            double bestLength = (preservedSurface.face[bestFace].V0(bestEdge)->P() - preservedSurface.face[bestFace].V1(bestEdge)->P()).Norm();

                            if (edgeLength >= bestLength) {
                                bestEdge = j;
                                bestComponent = cId;
                            }
                        }
                        else {
                            bestFace = i;
                            bestEdge = j;
                            bestComponent = cId;
                        }
                    }

                }
            }
        }

        assert(bestFace < std::numeric_limits<size_t>::max());
        assert(bestEdge < std::numeric_limits<size_t>::max());
        assert(bestComponent < std::numeric_limits<size_t>::max());
        assert(vcg::face::IsBorder(preservedSurface.face[bestFace], bestEdge));

        //Get edge
        typename PolyMeshType::CoordType firstV = preservedSurface.face[bestFace].P0(bestEdge);
        typename PolyMeshType::CoordType secondV = preservedSurface.face[bestFace].P1(bestEdge);
        if (firstV < secondV)
            std::swap(firstV, secondV);

        //Split faces
        typename PolyMeshType::CoordType newPoint =
                (preservedSurface.face[bestFace].V0(bestEdge)->P() + preservedSurface.face[bestFace].V1(bestEdge)->P())/2;

        vcg::tri::Allocator<PolyMeshType>::AddVertex(preservedSurface, newPoint);
        typename PolyMeshType::VertexType* newVert = &preservedSurface.vert.back();

        std::vector<typename PolyMeshType::VertexType*> newVertices(preservedSurface.face[bestFace].VN() + 1);
        for (size_t i = 0, j = 0; i < preservedSurface.face[bestFace].VN(); i++, j++) {
            newVertices[j] = preservedSurface.face[bestFace].V(i);

            if (i == bestEdge) {
                j++;
                newVertices[j] = newVert;
            }
        }

        preservedSurface.face[bestFace].Dealloc();
        preservedSurface.face[bestFace].Alloc(newVertices.size());
        for (size_t i = 0; i < newVertices.size(); i++) {
            preservedSurface.face[bestFace].V(i) = newVertices[i];
        }

        //Add to the map
        splittedMap.insert(std::make_pair(std::make_pair(firstV, secondV), newPoint));


        //Update status
        isComponentFeasible[bestComponent] = true;
        numNonFeasible--;

        std::cout << "Feasibility solver: face edge splitted." << std::endl;

        //Update for next iteration
        if (numNonFeasible > 0) {
            vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(preservedSurface);
            vcg::tri::UpdateFlags<PolyMeshType>::FaceBorderFromFF(preservedSurface);
            vcg::tri::UpdateFlags<PolyMeshType>::VertexBorderFromFaceAdj(preservedSurface);
        }

    } while (numNonFeasible > 0);

    //Split mapped edges
    splitEdgesMappedPoint(newSurface, splittedMap);

    return numNonFeasible;
}

template<class MeshType>
void splitEdgesMappedPoint(
        MeshType& newSurface,
        const std::map<std::pair<typename MeshType::CoordType, typename MeshType::CoordType>, typename MeshType::CoordType>& splittedMap)
{
    //Split edges in the new surface
    size_t numSplit=0;
    for (size_t i = 0; i < newSurface.face.size() && numSplit < splittedMap.size(); i++) {
       assert(newSurface.face[i].VN()==3);

       for (size_t j = 0; j < newSurface.face[i].VN(); j++) {
           if (vcg::face::IsBorder(newSurface.face[i], j)) {
               typename MeshType::CoordType firstV = newSurface.face[i].P0(j);
               typename MeshType::CoordType secondV = newSurface.face[i].P1(j);
               if (firstV < secondV)
                   std::swap(firstV, secondV);

               typename std::map<std::pair<typename MeshType::CoordType, typename MeshType::CoordType>, typename MeshType::CoordType>::const_iterator
                       it = splittedMap.find(std::make_pair(firstV, secondV));

               if (it != splittedMap.end()) {
                   numSplit++;

                   //Split the triangles
                   typename MeshType::CoordType newPoint = it->second;

                   vcg::tri::Allocator<MeshType>::AddVertex(newSurface, newPoint);
                   vcg::tri::Allocator<MeshType>::AddFaces(newSurface, 1);

                   typename MeshType::VertexType* oldV1 = newSurface.face[i].V1(j);
                   typename MeshType::VertexType* oldV2 = newSurface.face[i].V2(j);
                   newSurface.face[i].V1(j) = &newSurface.vert.back();
                   newSurface.face.back().V0(j) = &newSurface.vert.back();
                   newSurface.face.back().V1(j) = oldV1;
                   newSurface.face.back().V2(j) = oldV2;
               }
           }
       }
    }
    assert(numSplit == splittedMap.size());
}

template<class MeshType>
std::vector<std::set<std::pair<typename MeshType::CoordType, typename MeshType::CoordType>>> findComponentsBorderEdges(
        MeshType& mesh,
        const std::vector<std::vector<size_t>>& components)
{
    std::vector<std::set<std::pair<typename MeshType::CoordType, typename MeshType::CoordType>>>
            borders(components.size());

    for (size_t i = 0; i < components.size(); i++) {
        for (size_t j = 0; j < components[i].size(); j++) {
            size_t fId = components[i][j];
            assert(fId >= 0 && fId < mesh.face.size());

            for (size_t k = 0; k < mesh.face[fId].VN(); k++) {
                if (vcg::face::IsBorder(mesh.face[fId], k)) {
                    typename MeshType::CoordType first = mesh.face[fId].V0(k)->P();
                    typename MeshType::CoordType second = mesh.face[fId].V1(k)->P();

                    if (first < second)
                        std::swap(first, second);

                    borders[i].insert(
                        std::make_pair(
                            first,
                            second
                        )
                    );
                }
            }
        }
    }

    return borders;
}

template<class MeshType>
std::vector<bool> findComponentsFeasibility(
        const std::vector<std::set<std::pair<typename MeshType::CoordType,typename MeshType::CoordType>>>& borderEdges)
{
    std::vector<bool> isComponentFeasible(borderEdges.size());
    for (size_t i = 0; i < borderEdges.size(); i++) {
        isComponentFeasible[i] = (borderEdges[i].size() % 2 == 0);
    }
    return isComponentFeasible;
}

inline size_t countNonFeasible(
        const std::vector<bool>& isComponentFeasible)
{
    size_t numNonFeasible = 0;
    for (size_t i = 0; i < isComponentFeasible.size(); i++) {
        if (!isComponentFeasible[i]) {
            numNonFeasible++;
        }
    }
    return numNonFeasible;
}

template<class MeshType>
bool checkFeasibility(
        MeshType& mesh)
{
    //Find connected components
    std::vector<std::vector<size_t>> components =
            QuadRetopology::internal::findConnectedComponents<MeshType>(mesh);

    //Find border vertices
    std::vector<std::set<std::pair<typename MeshType::CoordType, typename MeshType::CoordType>>> borderEdges =
            findComponentsBorderEdges(mesh, components);

    //Find feasible components
    std::vector<bool> isComponentFeasible =
            findComponentsFeasibility<MeshType>(borderEdges);

    size_t numNonFeasible = countNonFeasible(isComponentFeasible);

    return numNonFeasible == 0;
}

template <class PolyMeshType, class TriangleMeshType>
bool checkBorderConsistency(
        PolyMeshType& preservedSurface,
        TriangleMeshType& newSurface)
{
    std::set<typename TriangleMeshType::CoordType> vertexSet1;
    for (size_t i = 0; i < newSurface.vert.size(); i++) {
        if (newSurface.vert[i].IsB()) {
            vertexSet1.insert(newSurface.vert[i].P());
        }
    }
    std::set<typename PolyMeshType::CoordType> vertexSet2;
    for (size_t i = 0; i < preservedSurface.vert.size(); i++) {
        if (preservedSurface.vert[i].IsB()) {
            vertexSet2.insert(preservedSurface.vert[i].P());
        }
    }
    return vertexSet1 == vertexSet2;
}

}
}
