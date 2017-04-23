#include "ControllerBase.h"
#include <GCL/Core/SceneGraph/SceneGraph.h>
#include <GCL/Core/SceneGraph/OpenMeshNode.h>
namespace GCL { namespace Utilities {

SceneGraph::OpenMeshNode *ControllerBase::getOpenMeshNode()
{
    return scenegraph_->getFirstDescendant<SceneGraph::OpenMeshNode* >(true);
}
int ControllerBase::getOpenMeshNodes(QList<SceneGraph::OpenMeshNode* > &list, bool is_selected)
{

    return scenegraph_->getDescendants<SceneGraph::OpenMeshNode* >(list,is_selected);
}



void ControllerBase::UnselectdNodes()
{
    if(getOpenMeshNode())
    {
        getOpenMeshNode()->setSelected(false);
    }

}


}}
