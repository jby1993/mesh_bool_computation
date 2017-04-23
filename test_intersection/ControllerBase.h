#ifndef CONTROLLERBASE_H
#define CONTROLLERBASE_H
#include <GCL/Core/Math/MathDefines.h>
#include <GCL/Core/SceneGraph/EventHandler/EventHandler.h>
namespace GCL{
namespace SceneGraph {
class SceneGraphNode;
class OpenMeshNode;
}
namespace Utilities {
class ControllerBase : public SceneGraph::EventHandler
{
public:
    ControllerBase(SceneGraph::SceneGraphNode* _scenegraph):scenegraph_(_scenegraph) {}
    virtual ~ControllerBase() {}
    virtual const char* getCategory() const {return getCategoryName();}
    static const char* getCategoryName() {return  "testOpenMeshAlgorithm";}
    const char* getEventHandlerName() const {return  "testOpenMeshAlgorithm";}


    SceneGraph::OpenMeshNode* getOpenMeshNode();
    int getOpenMeshNodes(QList<SceneGraph::OpenMeshNode* > &list, bool is_selected);
    void UnselectdNodes();
protected:
    SceneGraph::SceneGraphNode *scenegraph_;
};
}
}
#endif // CONTROLLERBASE_H
