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

#include "../Precompiled.h"

#include "../Core/Context.h"
#include "../Graphics/RibbonTrail.h"
#include "../Scene/Scene.h"
#include "../Scene/SceneEvents.h"
#include "../Resource/ResourceCache.h"
#include "../Graphics/VertexBuffer.h"
#include "../Graphics/IndexBuffer.h"
#include "../Scene/Node.h"
#include "../Graphics/Camera.h"
#include "../Graphics/Material.h"
#include "../Graphics/OctreeQuery.h"
#include "../Graphics/Geometry.h"
#include "../IO/Log.h"

namespace Urho3D
{

extern const char* GEOMETRY_CATEGORY;

inline bool CompareTails(Point* lhs, Point* rhs)
{
    return lhs->sortDistance_ > rhs->sortDistance_;
}

RibbonTrail::RibbonTrail(Context* context) : 
    Drawable(context, DRAWABLE_GEOMETRY),
    geometry_(new Geometry(context_)),
    animationLodBias_(1.0f),
    animationLodTimer_(0.0f),
    vertexBuffer_(new VertexBuffer(context_)),
    indexBuffer_(new IndexBuffer(context_)),
    bufferDirty_(true),
    previousPosition_(Vector3::ZERO),
    pointNum_(0),
    lifetime_(1.0f),
    tailLength_(0.1f),
    scale_(0.2f),
    tailTipColor_(Color(1.0f, 1.0f, 1.0f, 0.0f)),
    tailHeadColor_(Color(1.0f, 1.0f, 1.0f, 1.0f)),
    matchNode_(false),
    lastUpdateFrameNumber_(M_MAX_UNSIGNED),
    needUpdate_(false),
    sorted_(false),
    previousOffset_(Vector3::ZERO),
    forceUpdate_(false)
{
    geometry_->SetVertexBuffer(0, vertexBuffer_);
    geometry_->SetIndexBuffer(indexBuffer_);

    transforms_ = Matrix3x4::IDENTITY;

    batches_.Resize(1);
    batches_[0].geometry_ = geometry_;
    batches_[0].geometryType_ = GEOM_TRAIL;
    batches_[0].worldTransform_ = &transforms_;
    batches_[0].numWorldTransforms_ = 1;

    // for debug
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    SetMaterial(cache->GetResource<Material>("Materials/TailGenerator.xml"));
}

RibbonTrail::~RibbonTrail()
{
}

void RibbonTrail::RegisterObject(Context* context)
{
    context->RegisterFactory<RibbonTrail>(GEOMETRY_CATEGORY);

    URHO3D_ACCESSOR_ATTRIBUTE("Is Enabled", IsEnabled, SetEnabled, bool, true, AM_DEFAULT);
    URHO3D_COPY_BASE_ATTRIBUTES(Drawable);
	URHO3D_MIXED_ACCESSOR_ATTRIBUTE("Material", GetMaterialAttr, SetMaterialAttr, ResourceRef, ResourceRef(Material::GetTypeStatic()), AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Tail Lifetime", GetLifetime, SetLifetime, float, 1.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Tail Length", GetTailLength, SetTailLength, float, 0.1f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Width", GetWidthScale, SetWidthScale, float, 0.2f, AM_DEFAULT);
	URHO3D_ACCESSOR_ATTRIBUTE("Start Color", GetColorForHead, SetColorForHead, Color, Color::WHITE, AM_DEFAULT);
	URHO3D_ACCESSOR_ATTRIBUTE("End Color", GetColorForTip, SetColorForTip, Color, Color::WHITE, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Animation LOD Bias", GetAnimationLodBias, SetAnimationLodBias, float, 1.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Match Node Rotation", GetMatchNodeOrientation, SetMatchNodeOrientation, bool, false, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Sort By Distance", IsSorted, SetSorted, bool, false, AM_DEFAULT);
}

void RibbonTrail::ProcessRayQuery(const RayOctreeQuery& query, PODVector<RayQueryResult>& results)
{
    // If no billboard-level testing, use the Drawable test
    if (query.level_ < RAY_TRIANGLE)
    {
        Drawable::ProcessRayQuery(query, results);
        return;
    }

    // Check ray hit distance to AABB before proceeding with trail-level tests
    if (query.ray_.HitDistance(GetWorldBoundingBox()) >= query.maxDistance_)
        return;

    // Approximate the tails as spheres for raycasting
    for (unsigned i = 0; i < points_.Size() - 1; ++i)
    {
        Vector3 center = (points_[i].position_ + points_[i+1].position_) * 0.5f;
        Vector3 scale = scale_ * Vector3::ONE;
        // Tail should be represented in cylinder shape, but we don't have this yet on Urho,
        // so this implementation will use bounding box instead (hopefully only temporarily)
        float distance = query.ray_.HitDistance(BoundingBox(center - scale, center + scale));
        if (distance < query.maxDistance_)
        {
            // If the code reaches here then we have a hit
            RayQueryResult result;
            result.position_ = query.ray_.origin_ + distance * query.ray_.direction_;
            result.normal_ = -query.ray_.direction_;
            result.distance_ = distance;
            result.drawable_ = this;
            result.node_ = node_;
            result.subObject_ = i;
            results.Push(result);
        }
    }
}

void RibbonTrail::OnSetEnabled()
{
    Drawable::OnSetEnabled();

    previousPosition_ = node_->GetWorldPosition();

    Scene* scene = GetScene();
    if (scene)
    {
        if (IsEnabledEffective())
            SubscribeToEvent(scene, E_SCENEPOSTUPDATE, URHO3D_HANDLER(RibbonTrail, HandleScenePostUpdate));
        else
            UnsubscribeFromEvent(scene, E_SCENEPOSTUPDATE);
    }
}

void RibbonTrail::HandleScenePostUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace ScenePostUpdate;
    lastTimeStep_ = eventData[P_TIMESTEP].GetFloat();

    // Update if frame has changed
    if (viewFrameNumber_ != lastUpdateFrameNumber_)
    {
        // Reset if ribbon trail is too small and too much difference in frame
        if(points_.Size() < 3 && viewFrameNumber_ - lastUpdateFrameNumber_ > 1)
        {
            previousPosition_ = node_->GetWorldPosition();
            points_.Erase(0, points_.Size());
        }

        lastUpdateFrameNumber_ = viewFrameNumber_;
        needUpdate_ = true;
        MarkForUpdate();
    }
}

void RibbonTrail::Update(const FrameInfo &frame)
{
    Drawable::Update(frame);

    if (!needUpdate_)
        return;

    UpdateTail();
    OnMarkedDirty(node_);
    needUpdate_ = false;
}

void RibbonTrail::UpdateTail()
{
    Vector3 worldPosition = node_->GetWorldPosition();
    float path = (previousPosition_ - worldPosition).Length();

    // Update tails lifetime
    int expiredIndex = -1;
    if (points_.Size() > 0)
    {
        // No need to update last point
        for (unsigned i = 0; i < points_.Size() - 1; ++i)
        {
            points_[i].lifetime_ += lastTimeStep_;

            // Get point index with expired lifetime
            if(points_[i].lifetime_ > lifetime_)
                expiredIndex = i;
        }
    }

    // Delete expired points
    if(expiredIndex != -1)
    {
        points_.Erase(0, expiredIndex+1);

        // Update endTail pointer
        if(points_.Size() > 1)
        {
            endTail_.position_ = points_[0].position_;
            startEndTailTime_ = points_[0].lifetime_;
        }
    }

    // Delete lonely point
    if(points_.Size() == 1)
    {
        points_.Erase(0, 1);
        previousPosition_ = worldPosition;
    }

    // Update endtail position
    if (points_.Size() > 1 && points_[0].lifetime_ < lifetime_)
    {
        float step = SmoothStep(startEndTailTime_, lifetime_, points_[0].lifetime_);
        points_[0].position_ = Lerp(endTail_.position_, points_[1].position_, step);
        bufferDirty_ = true;
    }

    // Add starting points
    //if(points_.Size() == 0 && path > 0.01f)
    if(points_.Size() == 0 && path > M_EPSILON)
    {
        Vector3 forwardmotion = matchNode_ ?
                    GetNode()->GetWorldDirection() : (previousPosition_ - worldPosition).Normalized();

        Point startPoint;
        startPoint.position_ = previousPosition_;
        startPoint.lifetime_ = 0.0f;
        startPoint.forward_ = forwardmotion;

        Point nextPoint;
        nextPoint.position_ = worldPosition;
        nextPoint.lifetime_ = 0.0f;
        nextPoint.forward_ = forwardmotion;

        points_.Push(startPoint);
        points_.Push(nextPoint);

        // Update endtail
        endTail_.position_ = startPoint.position_;
        startEndTailTime_ = 0.0f;
    }

    // Add more points
    if (points_.Size() > 1)
    {
        Vector3 forwardmotion = matchNode_ ?
                    GetNode()->GetWorldDirection() : (previousPosition_ - worldPosition).Normalized();

        // Add more point if path exceeded tail length
        if(path > tailLength_)
        {
            Point newPoint;
            newPoint.position_ = worldPosition;
            newPoint.lifetime_ = 0.0f;
            newPoint.forward_ = forwardmotion;
            points_.Push(newPoint);

            previousPosition_ = worldPosition;
        }
        else
        {
            // Updating recent tail
            points_.Back().position_ = worldPosition;
            if(forwardmotion != Vector3::ZERO)
                points_.Back().forward_ = forwardmotion;
        }
    }

    // Update buffer size if size of points different with tail number
    if (points_.Size() != pointNum_)
        bufferSizeDirty_ = true;

}

void RibbonTrail::UpdateBatches(const FrameInfo& frame) 
{
    // Update information for renderer about this drawable
    distance_ = frame.camera_->GetDistance(GetWorldBoundingBox().Center());
    batches_[0].distance_ = distance_;

    // Calculate scaled distance for animation LOD
    float scale = GetWorldBoundingBox().Size().DotProduct(DOT_SCALE);
    // If there are no billboards, the size becomes zero, and LOD'ed updates no longer happen. Disable LOD in that case
    if (scale > M_EPSILON)
        lodDistance_ = frame.camera_->GetLodDistance(distance_, scale, lodBias_);
    else
        lodDistance_ = 0.0f;

    Vector3 worldPos = node_->GetWorldPosition();
    Vector3 offset = (worldPos - frame.camera_->GetNode()->GetWorldPosition());
    if (sorted_ && offset != previousOffset_)
    {
        bufferDirty_ = true;
        previousOffset_ = offset;
    }
}

void RibbonTrail::UpdateGeometry(const FrameInfo& frame)
{
    if (bufferSizeDirty_ || indexBuffer_->IsDataLost())
        UpdateBufferSize();

    if (bufferDirty_ || vertexBuffer_->IsDataLost())
        UpdateVertexBuffer(frame);
}

UpdateGeometryType RibbonTrail::GetUpdateGeometryType()
{	
    if (bufferDirty_ || bufferSizeDirty_ || vertexBuffer_->IsDataLost() || indexBuffer_->IsDataLost())
        return UPDATE_MAIN_THREAD;
    else
        return UPDATE_NONE;
}

void RibbonTrail::SetMaterial(Material* material)
{
    batches_[0].material_ = material;
    MarkNetworkUpdate();
}

void RibbonTrail::OnSceneSet(Scene* scene)
{
    Drawable::OnSceneSet(scene);

    if (scene && IsEnabledEffective())
        SubscribeToEvent(scene, E_SCENEPOSTUPDATE, URHO3D_HANDLER(RibbonTrail, HandleScenePostUpdate));
    else if (!scene)
         UnsubscribeFromEvent(E_SCENEPOSTUPDATE);
}

void RibbonTrail::OnWorldBoundingBoxUpdate() 
{
    BoundingBox worldBox;

    for (unsigned i = 0; i < points_.Size(); ++i)
    {
        Vector3 &p = points_[i].position_;
        Vector3 scale = scale * Vector3::ONE;
        worldBox.Merge(BoundingBox(p - scale, p + scale));
    }

    worldBoundingBox_ = worldBox;
}

void RibbonTrail::UpdateBufferSize() 
{
    pointNum_ = points_.Size();

    if (vertexBuffer_->GetVertexCount() != (pointNum_ * 4))
        vertexBuffer_->SetSize((pointNum_ * 4),
            MASK_POSITION | MASK_NORMAL | MASK_COLOR | MASK_TEXCOORD1 | MASK_TANGENT, true);

    if (indexBuffer_->GetIndexCount() != ((pointNum_ - 1) * 6))
        indexBuffer_->SetSize(((pointNum_ - 1) * 6), false);

    bufferSizeDirty_ = false;
    bufferDirty_ = true;
    forceUpdate_ = true;

    if (pointNum_ == 0)
        return;

    // Indices do not change for a given tail generator capacity
    unsigned short* dest = (unsigned short*)indexBuffer_->Lock(0, ((pointNum_ - 1) * 6), true);
    if (!dest)
        return;

    unsigned vertexIndex = 0;
    unsigned stripsLen = pointNum_ - 1;
    while (stripsLen--)
    {
        dest[0] = (unsigned short)vertexIndex;
        dest[1] = (unsigned short)(vertexIndex + 3);
        dest[2] = (unsigned short)(vertexIndex + 1);
        dest[3] = (unsigned short)vertexIndex;
        dest[4] = (unsigned short)(vertexIndex + 3);
        dest[5] = (unsigned short)(vertexIndex + 2);

        dest += 6;
        vertexIndex += 4;
    }

    indexBuffer_->Unlock();
    indexBuffer_->ClearDataLost();
}

void RibbonTrail::UpdateVertexBuffer(const FrameInfo& frame) 
{
    // If using animation LOD, accumulate time and see if it is time to update
    if (animationLodBias_ > 0.0f && lodDistance_ > 0.0f)
    {
        animationLodTimer_ += animationLodBias_ * frame.timeStep_ * ANIMATION_LOD_BASESCALE;
        if (animationLodTimer_ >= lodDistance_)
            animationLodTimer_ = fmodf(animationLodTimer_, lodDistance_);
        else
        {
            // No LOD if immediate update forced
            if (!forceUpdate_)
                return;
        }
    }

    unsigned numPoints = points_.Size();

    // if tail path is short and nothing to draw, exit
    if (numPoints < 2)
    {
        batches_[0].geometry_->SetDrawRange(TRIANGLE_LIST, 0, 0, false);
        return;
    }

    // Sort points
    sortedPoints_.Resize(numPoints);

    for (unsigned i = 0; i < numPoints; ++i)
    {
        Point& point = points_[i];
        sortedPoints_[i] = &point;
        if (sorted_)
            point.sortDistance_ = frame.camera_->GetDistanceSquared(point.position_);
    }

    if (sorted_)
        Sort(sortedPoints_.Begin(), sortedPoints_.End(), CompareTails);

    // Clear previous mesh data
    tailMesh_.Clear();

    // Update individual trail elapsed length
    float trailLength = 0.0f;
    for(unsigned i = 0; i < numPoints; ++i)
    {
        float length = i == 0 ? 0.0f : (points_[i].position_ - points_[i-1].position_).Length();
        trailLength += length;
        points_[i].elapsedLength_ = trailLength;
        if(i < numPoints - 1)
            points_[i].next_ = &points_[i+1];
    }

    // generate strips of tris
    TailVertex v;

    // Forward part of tail (strip in xz plane)
    for (unsigned i = 0; i < numPoints; ++i)
    {
        Point& point = *sortedPoints_[i];

        if (sortedPoints_[i] == &points_.Back()) continue;

        float factor = SmoothStep(0.0f, trailLength, point.elapsedLength_);

        Color c = tailTipColor_.Lerp(tailHeadColor_, factor);
        v.color_ = c.ToUInt();
        v.direction_ = Vector4(point.forward_, 0.0f);
        v.position_ = point.position_;

        v.uv_ = Vector2(factor, 0.0f);
        v.scale_ = Vector3::ONE * scale_;
        tailMesh_.Push(v);

        v.uv_ = Vector2(factor, 1.0f);
        v.scale_ = Vector3::ONE * -scale_;
        tailMesh_.Push(v);

        // Next tail factor
        factor = SmoothStep(0.0f, trailLength, point.next_->elapsedLength_);

        // Next tail
        c = tailTipColor_.Lerp(tailHeadColor_, factor);
        v.color_ = c.ToUInt();
        v.direction_ = Vector4(point.next_->forward_, 0.0f);
        v.position_ = point.next_->position_;

        v.uv_ = Vector2(factor, 0.0f);
        v.scale_ = Vector3::ONE * scale_;
        tailMesh_.Push(v);

        v.uv_ = Vector2(factor, 1.0f);
        v.scale_ = Vector3::ONE * -scale_;
        tailMesh_.Push(v);

    }

    // copy new mesh to vertex buffer
    unsigned meshVertexCount = tailMesh_.Size();
    batches_[0].geometry_->SetDrawRange(TRIANGLE_LIST, 0, (numPoints - 1) * 6, false);
    bufferDirty_ = false;
    forceUpdate_ = false;

    // get pointer
    TailVertex* dest = (TailVertex*)vertexBuffer_->Lock(0, meshVertexCount, true);
    if (!dest)
        return;
    // copy to vertex buffer
    memcpy(dest, &tailMesh_[0], meshVertexCount * sizeof(TailVertex));

    vertexBuffer_->Unlock();
    vertexBuffer_->ClearDataLost();
}

void RibbonTrail::SetLifetime(float time)
{
    lifetime_ = time;
    Commit();
}

void RibbonTrail::SetTailLength(float length) 
{
    tailLength_ = length;
    Commit();
}

void RibbonTrail::SetColorForTip(const Color& c)
{
    tailTipColor_ = Color(c.r_, c.g_, c.b_, c.a_);
    Commit();
}

void RibbonTrail::SetColorForHead(const Color& c)
{
    tailHeadColor_ = Color(c.r_, c.g_, c.b_, c.a_);
    Commit();
}

void RibbonTrail::SetMatchNodeOrientation(bool value)
{
    matchNode_ = value;
    Commit();
}

void RibbonTrail::SetSorted(bool enable)
{
    sorted_ = enable;
    Commit();
}

void RibbonTrail::SetMaterialAttr(const ResourceRef& value)
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    SetMaterial(cache->GetResource<Material>(value.name_));
    Commit();
}

void RibbonTrail::SetWidthScale(float scale)
{
    scale_ = scale;
    Commit();
}

void RibbonTrail::SetAnimationLodBias(float bias)
{
    animationLodBias_ = Max(bias, 0.0f);
    MarkNetworkUpdate();
}

void RibbonTrail::Commit()
{
    MarkPositionsDirty();
    MarkNetworkUpdate();
}

void RibbonTrail::MarkPositionsDirty()
{
    Drawable::OnMarkedDirty(node_);
    bufferDirty_ = true;
}

ResourceRef RibbonTrail::GetMaterialAttr() const
{
    return GetResourceRef(batches_[0].material_, Material::GetTypeStatic());
}

}
