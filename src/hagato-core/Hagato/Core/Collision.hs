-----------------------------------------------------------------------------
-- |
-- Module      : Hagato.Core.Collision
-- Copyright   : (c) Michael Szvetits, 2023
-- License     : BSD-3-Clause (see the file LICENSE)
-- Maintainer  : typedbyte@qualified.name
-- Stability   : stable
-- Portability : portable
--
-- Types and functions for collision detection.
-----------------------------------------------------------------------------
module Hagato.Core.Collision
  ( intersectAABB
  , transformAABB
  ) where

import Hagato.Core.Math.Mat4 (Mat4, multiplyVector)
import Hagato.Core.Math.Ray  (Ray(..))
import Hagato.Core.Math.Vec3 (Vec3(..))
import Hagato.Core.Math.Vec4 (Vec4(..))

-- | Tests if a ray intersects an axis-aligned bounding box.
-- Returns the distance along the ray to the intersection point, if any.
-- Uses the slab method for efficient ray-AABB intersection testing.
intersectAABB :: Ray -> (Vec3, Vec3) -> Maybe Float
intersectAABB (Ray (Vec3 ox oy oz) (Vec3 dx dy dz)) (Vec3 minX minY minZ, Vec3 maxX maxY maxZ) =
  let
    -- Compute inverse direction to avoid division in the loop
    invDx = if dx == 0 then 1e30 else 1 / dx
    invDy = if dy == 0 then 1e30 else 1 / dy
    invDz = if dz == 0 then 1e30 else 1 / dz
    
    -- Compute intersection distances for X axis
    tx1 = (minX - ox) * invDx
    tx2 = (maxX - ox) * invDx
    tMinX = min tx1 tx2
    tMaxX = max tx1 tx2
    
    -- Compute intersection distances for Y axis
    ty1 = (minY - oy) * invDy
    ty2 = (maxY - oy) * invDy
    tMinY = min ty1 ty2
    tMaxY = max ty1 ty2
    
    -- Compute intersection distances for Z axis
    tz1 = (minZ - oz) * invDz
    tz2 = (maxZ - oz) * invDz
    tMinZ = min tz1 tz2
    tMaxZ = max tz1 tz2
    
    -- Find the farthest entry point and nearest exit point
    tMin = maximum [tMinX, tMinY, tMinZ]
    tMax = minimum [tMaxX, tMaxY, tMaxZ]
  in
    -- Check if there's a valid intersection
    if tMax >= tMin && tMax >= 0
      then Just (max 0 tMin)  -- Return entry distance, clamped to 0 if ray starts inside
      else Nothing

-- | Transforms an axis-aligned bounding box from local space to world space
-- by transforming all 8 corners and computing the new min/max bounds.
transformAABB :: Mat4 -> (Vec3, Vec3) -> (Vec3, Vec3)
transformAABB transform (Vec3 minX minY minZ, Vec3 maxX maxY maxZ) =
  let
    -- Generate all 8 corners of the AABB
    corners =
      [ Vec4 minX minY minZ 1
      , Vec4 maxX minY minZ 1
      , Vec4 minX maxY minZ 1
      , Vec4 maxX maxY minZ 1
      , Vec4 minX minY maxZ 1
      , Vec4 maxX minY maxZ 1
      , Vec4 minX maxY maxZ 1
      , Vec4 maxX maxY maxZ 1
      ]
    
    -- Transform all corners
    transformedCorners = map (transform `multiplyVector`) corners
    
    -- Extract Vec3 from Vec4 (ignoring w component)
    toVec3 (Vec4 x y z _) = Vec3 x y z
    vec3Corners = map toVec3 transformedCorners
    
    -- Find min and max across all transformed corners
    findMin (Vec3 x1 y1 z1) (Vec3 x2 y2 z2) = Vec3 (min x1 x2) (min y1 y2) (min z1 z2)
    findMax (Vec3 x1 y1 z1) (Vec3 x2 y2 z2) = Vec3 (max x1 x2) (max y1 y2) (max z1 z2)
    
    newMin = foldr1 findMin vec3Corners
    newMax = foldr1 findMax vec3Corners
  in
    (newMin, newMax)
