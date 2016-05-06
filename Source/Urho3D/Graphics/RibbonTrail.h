//
// Copyright (c) 2008-2016 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#pragma once

#include "../Graphics/Drawable.h"

namespace Urho3D
{

class IndexBuffer;
class VertexBuffer;

/// Vertex struct for tail  
struct URHO3D_API TailVertex
{
    Vector3 position_;
    Vector3 scale_;
    unsigned color_;
    Vector2 uv_;
    Vector4 direction_;
};

/// Trail is consisting of series of point
struct URHO3D_API Point
{
    Vector3 position_;
    Vector3 forward_;
    float elapsedLength_;
    Point* next_;

    float lifetime_;
    float sortDistance_;
};

static const unsigned MAX_TAILS = 65536 / 6;

/// Custom component that creates a tail
class URHO3D_API RibbonTrail : public Drawable
{
	URHO3D_OBJECT(RibbonTrail, Drawable);

public:
    /// Construct.
    RibbonTrail(Context* context);
    /// Destruct.
    virtual ~RibbonTrail();
    /// Register object factory.
    static void RegisterObject(Context* context);
    /// Process octree raycast. May be called from a worker thread.
    virtual void ProcessRayQuery(const RayOctreeQuery& query, PODVector<RayQueryResult>& results);
    /// Handle enabled/disabled state change.
    virtual void OnSetEnabled();
    ///
    virtual void Update(const FrameInfo &frame);
    /// Calculate distance and prepare batches for rendering. May be called from worker thread(s), possibly re-entrantly.
    virtual void UpdateBatches(const FrameInfo& frame);
    /// Prepare geometry for rendering. Called from a worker thread if possible (no GPU update.)
    virtual void UpdateGeometry(const FrameInfo& frame);
    /// Return whether a geometry update is necessary, and if it can happen in a worker thread.
    virtual UpdateGeometryType GetUpdateGeometryType();

    /// Set material.
    void SetMaterial(Material* material);
    /// Set tail segment length
    void SetTailLength(float length);
    /// Get tail segment length
    float GetTailLength();
    /// Set count segments of all tail 
    //void SetNumTails(unsigned num);
    /// Set width scale of the tail
    void SetWidthScale(float scale);
    /// Set vertex blended color for tip of all tail.
    void SetColorForTip(const Color& c);
    // Set vertex blended color for head of all tail.
    void SetColorForHead(const Color& c);
    /// Set material attribute.
    void SetMaterialAttr(const ResourceRef& value);
    /// Return material attribute.
    ResourceRef GetMaterialAttr() const;
    /// Set whether billboards are sorted by distance. Default false.
    void SetSorted(bool enable);
    ///
    void SetLifetime(float time);
    /// Set animation LOD bias.
    void SetAnimationLodBias(float bias);
    /// Mark for bounding box and vertex buffer update. Call after modifying the trails.
    void Commit();

    /// Get whether or not this tail is matching node direction vectors
    bool GetMatchNodeOrientation() const { return matchNode_; }
    /// Set whether or not this tail is matching node direction vectors
    void SetMatchNodeOrientation(bool value);
    ///
    //void SetWidthChange(float widthDeltaPerSecond);
	///
    float GetWidthScale() const { return scale_; }
    /// Get count segments of all tail
    //unsigned  GetNumTails() const { return tailNum_; }
    ///
    float GetTailLength() const { return tailLength_;  }
    ///
    const Color& GetColorForHead() const { return tailHeadColor_; }
    ///
    const Color& GetColorForTip() const { return tailTipColor_;  }
    ///
    //float GetWidthChange() const {return deltaWidth_;}
    /// Return whether tails are sorted.
    bool IsSorted() const { return sorted_; }
    ///
    float GetLifetime() const {return lifetime_;}
    /// Return animation LOD bias.
    float GetAnimationLodBias() const { return animationLodBias_; }

protected:
    /// Handle node being assigned.
    virtual void OnSceneSet(Scene* scene);
    /// Recalculate the world-space bounding box.
    virtual void OnWorldBoundingBoxUpdate();
    /// Mark vertex buffer to need an update.
    void MarkPositionsDirty();
    /// Tails.
    PODVector<Point> points_;
    /// Tails sorted flag.
    bool sorted_;
    /// Animation LOD bias.
    float animationLodBias_;
    /// Animation LOD timer.
    float animationLodTimer_;

private:
    /// Handle scene post-update event.
    void HandleScenePostUpdate(StringHash eventType, VariantMap& eventData);

    /// Resize RibbonTrail vertex and index buffers.
    void UpdateBufferSize();
    /// Rewrite RibbonTrail vertex buffer.
    void UpdateVertexBuffer(const FrameInfo& frame);
	/// Update/Rebuild tail mesh only if position changed (called by UpdateBatches())
	void UpdateTail();
    /// Geometry.
    SharedPtr<Geometry> geometry_;
    /// Vertex buffer.
    SharedPtr<VertexBuffer> vertexBuffer_;
    /// Index buffer.
    SharedPtr<IndexBuffer> indexBuffer_;
    /// Transform matrices for position and orientation.
    Matrix3x4 transforms_;
    /// Buffers need resize flag.
    bool bufferSizeDirty_;
    /// Vertex buffer needs rewrite flag.
    bool bufferDirty_;
	///
	bool matchNode_;
	///
    /// Previous position of tail
    Vector3 previousPosition_;
    ///
	float tailLength_;
    ///
	float scale_;
	///
    //unsigned tailNum_;
    ///
    unsigned pointNum_;
	///
    Vector<TailVertex> tailMesh_;
	///
    Color tailTipColor_;
    ///
    Color tailHeadColor_;
    /// Last scene timestep.
    float lastTimeStep_;
    /// Delta width
    //float deltaWidth_;
    // Lifetime
    float lifetime_;
    /// Rendering framenumber on which was last updated.
    unsigned lastUpdateFrameNumber_;
    /// Need update flag.
    bool needUpdate_;
    /// Sorting flag. Triggers a vertex buffer rewrite for each view this tails is rendered from.
    //bool sortThisFrame_;
    /// Frame number on which was last sorted.
    unsigned sortFrameNumber_;
    /// Previous offset to camera for determining whether sorting is necessary.
    Vector3 previousOffset_;
    /// Billboard pointers for sorting.
    Vector<Point*> sortedPoints_;
    /// Force update flag (ignore animation LOD momentarily.)
    bool forceUpdate_;

    Point endTail_;
    float startEndTailTime_;
};

}
