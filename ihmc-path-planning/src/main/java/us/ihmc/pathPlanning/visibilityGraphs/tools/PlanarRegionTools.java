package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.signedDistanceFromPoint3DToPlane3D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegion;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.lists.ListWrappingIndexTools;

public class PlanarRegionTools
{
   /**
    * Finds and returns the closest point the the provided point on the planar regions.
    */
   public static Point3D projectPointToPlanes(Point3DReadOnly point, PlanarRegionsList regions)
   {
      double smallestDistance = Double.POSITIVE_INFINITY;
      Point3D closestPoint = null;

      for (PlanarRegion region : regions.getPlanarRegionsAsList())
      {
         Point3D intersection = closestPointOnPlane(point, region);
         double distance = intersection.distance(point);

         if (closestPoint == null || distance < smallestDistance)
         {
            smallestDistance = distance;
            closestPoint = intersection;
         }
      }

      return closestPoint;
   }

   /**
    * Projects the given point onto the planar region, returning the closest point on the region to
    * the provided point.
    */
   public static Point3D closestPointOnPlane(Point3DReadOnly point, PlanarRegion region)
   {
      RigidBodyTransform regionToWorld = new RigidBodyTransform();
      region.getTransformToWorld(regionToWorld);

      Vector3D planeNormal = new Vector3D(0.0, 0.0, 1.0);
      planeNormal.applyTransform(regionToWorld);

      Point3D pointOnPlane = new Point3D();
      pointOnPlane.set(region.getConvexPolygon(0).getVertex(0));
      pointOnPlane.applyTransform(regionToWorld);

      Point3D intersectionWithPlane = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, point, planeNormal);
      if (intersectionWithPlane == null)
      {
         return null;
      }

      Point3D intersectionInPlaneFrame = new Point3D(intersectionWithPlane);
      intersectionInPlaneFrame.applyInverseTransform(regionToWorld);
      Point2D intersectionInPlaneFrame2D = new Point2D(intersectionInPlaneFrame);

      // checking convex hull here - might be better to check all polygons to avoid false positive
      ConvexPolygon2D convexHull = region.getConvexHull();
      if (!convexHull.isPointInside(intersectionInPlaneFrame2D))
      {
         convexHull.orthogonalProjection(intersectionInPlaneFrame2D);
         intersectionWithPlane.setToZero();
         intersectionWithPlane.set(intersectionInPlaneFrame2D);
         intersectionWithPlane.applyTransform(regionToWorld);
      }
      return intersectionWithPlane;
   }

   /**
    * Projects the given point onto a planar region from the list. The projection is done along the
    * z axis in world frame and if there is multiple regions that the point can be projected onto,
    * the highest intersection point will be returned.
    * <p>
    * Will return null if the is no planar region above or below the point.
    */
   public static Point3D projectPointToPlanesVertically(Point3DReadOnly point, PlanarRegionsList regions)
   {
      return projectPointToPlanesVertically(point, regions.getPlanarRegionsAsList());
   }

   /**
    * Projects the given point onto a planar region from the list. The projection is done along the
    * z axis in world frame and if there is multiple regions that the point can be projected onto,
    * the highest intersection point will be returned.
    * <p>
    * Will return null if the is no planar region above or below the point.
    */
   public static Point3D projectPointToPlanesVertically(Point3DReadOnly point, List<PlanarRegion> regions)
   {
      Line3D projectionLine = new Line3D(point, new Vector3D(0.0, 0.0, 1.0));
      Point3D highestIntersection = null;

      for (PlanarRegion region : regions)
      {
         Point3D intersection = intersectRegionWithLine(region, projectionLine);

         if (intersection == null)
         {
            continue;
         }

         if (highestIntersection == null || highestIntersection.getZ() < intersection.getZ())
         {
            highestIntersection = intersection;
         }
      }

      return highestIntersection;
   }

   /**
    * Will return the intersection point between a line and a single planar region. If the line does
    * not intersect the region this method will return null.
    */
   public static Point3D intersectRegionWithLine(PlanarRegion region, Line3D projectionLine)
   {
      RigidBodyTransform regionToWorld = new RigidBodyTransform();
      region.getTransformToWorld(regionToWorld);

      Vector3D planeNormal = new Vector3D(0.0, 0.0, 1.0);
      planeNormal.applyTransform(regionToWorld);

      Point3D pointOnPlane = new Point3D();
      pointOnPlane.set(region.getConvexPolygon(0).getVertex(0));
      pointOnPlane.applyTransform(regionToWorld);

      Point3DReadOnly pointOnLine = projectionLine.getPoint();
      Vector3DReadOnly directionOfLine = projectionLine.getDirection();
      Point3D intersectionWithPlane = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, pointOnLine, directionOfLine);
      if (intersectionWithPlane == null)
      {
         return null;
      }

      // checking convex hull here - might be better to check all polygons to avoid false positive
      if (isPointInWorldInsidePlanarRegion(region, intersectionWithPlane))
      {
         return intersectionWithPlane;
      }

      return null;
   }

   public static Point3D intersectRegionsWithRay(PlanarRegionsList regions, Point3D rayStart, Vector3D rayDirection)
   {
      double smallestDistance = Double.POSITIVE_INFINITY;
      Point3D closestIntersection = null;

      for (PlanarRegion region : regions.getPlanarRegionsAsList())
      {
         Point3D intersection = intersectRegionWithRay(region, rayStart, rayDirection);
         if (intersection == null)
         {
            continue;
         }
         double distance = intersection.distance(rayStart);
         if (distance < smallestDistance)
         {
            smallestDistance = distance;
            closestIntersection = intersection;
         }
      }

      return closestIntersection;
   }

   public static Point3D intersectRegionWithRay(PlanarRegion region, Point3D rayStart, Vector3D rayDirection)
   {
      RigidBodyTransform regionToWorld = new RigidBodyTransform();
      region.getTransformToWorld(regionToWorld);

      Vector3D planeNormal = new Vector3D(0.0, 0.0, 1.0);
      planeNormal.applyTransform(regionToWorld);

      Point3D pointOnPlane = new Point3D();
      pointOnPlane.set(region.getConvexPolygon(0).getVertex(0));
      pointOnPlane.applyTransform(regionToWorld);

      Point3D intersectionWithPlane = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, rayStart, rayDirection);
      if (intersectionWithPlane == null)
      {
         return null;
      }

      Point3D intersectionInPlaneFrame = new Point3D(intersectionWithPlane);
      intersectionInPlaneFrame.applyInverseTransform(regionToWorld);
      // checking convex hull here - might be better to check all polygons to avoid false positive
      if (!region.getConvexHull().isPointInside(intersectionInPlaneFrame.getX(), intersectionInPlaneFrame.getY()))
      {
         return null;
      }

      Vector3D rayToIntersection = new Vector3D();
      rayToIntersection.sub(intersectionWithPlane, rayStart);
      if (rayToIntersection.dot(rayDirection) < 0.0)
      {
         return null;
      }

      return intersectionWithPlane;
   }

   public static boolean isPointOnRegion(PlanarRegion region, Point3D point, double epsilon)
   {
      Point3D closestPoint = closestPointOnPlane(point, region);
      return closestPoint.epsilonEquals(point, epsilon);
   }

   public static boolean isPointInsideAnyRegion(List<PlanarRegion> regions, Point3DReadOnly pointToCheck)
   {
      for (PlanarRegion region : regions)
      {
         if (isPointInWorldInsidePlanarRegion(region, pointToCheck))
         {
            return true;
         }
      }

      return false;
   }

   public static PlanarRegion getRegionContainingThisPoint(Point3D point, List<PlanarRegion> regions)
   {
      for (PlanarRegion region : regions)
      {
         if (isPointInWorldInsidePlanarRegion(region, point))
         {
            return region;
         }
      }

      return null;
   }

   public static NavigableRegion getNavigableRegionContainingThisPoint(Point3DReadOnly point, List<NavigableRegion> navigableRegions)
   {
      List<NavigableRegion> containers = new ArrayList<>();

      for (NavigableRegion navigableRegion : navigableRegions)
      {
         if (isPointInWorldInsidePlanarRegion(navigableRegion.getHomeRegion(), point))
         {
            containers.add(navigableRegion);
         }
      }

      if (containers.isEmpty())
         return null;
      if (containers.size() == 1)
         return containers.get(0);

      Point3D pointOnRegion = new Point3D();
      Vector3D regionNormal = new Vector3D();

      NavigableRegion closestContainer = containers.get(0);
      closestContainer.getHomeRegion().getNormal(regionNormal);
      closestContainer.getHomeRegion().getPointInRegion(pointOnRegion);
      double minDistance = EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, pointOnRegion, regionNormal);

      for (int i = 1; i < containers.size(); i++)
      {
         NavigableRegion candidate = containers.get(i);
         candidate.getHomeRegion().getNormal(regionNormal);
         candidate.getHomeRegion().getPointInRegion(pointOnRegion);
         double distance = EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, pointOnRegion, regionNormal);
         if (distance < minDistance)
         {
            closestContainer = candidate;
            minDistance = distance;
         }
      }

      return closestContainer;
   }

   public static boolean isPointInWorldInsidePlanarRegion(PlanarRegion region, Point3DReadOnly pointInWorldToCheck)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      region.getTransformToWorld(transformToWorld);
      Point3D pointInLocalToCheck = new Point3D();
      transformToWorld.inverseTransform(pointInWorldToCheck, pointInLocalToCheck);
      return isPointInLocalInsidePlanarRegion(region, pointInLocalToCheck);
   }

   public static boolean isPointInLocalInsidePlanarRegion(PlanarRegion region, Point3DReadOnly pointInLocalToCheck)
   {
      return isPointInLocalInsidePlanarRegion(region, new Point2D(pointInLocalToCheck));
   }

   public static boolean isPointInLocalInsidePlanarRegion(PlanarRegion planarRegion, Point2DReadOnly pointInLocalToCheck)
   {
      if (!planarRegion.getConvexHull().getBoundingBox().isInsideInclusive(pointInLocalToCheck))
         return false;
      return isPointInsidePolygon(planarRegion.getConcaveHull(), pointInLocalToCheck);
   }

   public static boolean isPointInsidePolygon(Point2DReadOnly[] polygon, Point2DReadOnly pointToCheck)
   {
      return isPointInsidePolygon(Arrays.asList(polygon), pointToCheck);
   }

   public static boolean isPointInsidePolygon(List<? extends Point2DReadOnly> polygon, Point2DReadOnly pointToCheck)
   {
      if (polygon.size() < 3)
      {
         return false;
      }

      Point2DReadOnly rayOrigin = pointToCheck;
      Vector2D rayDirection = new Vector2D();

      Point2D pointOnArbitraryEdge = new Point2D();

      for (int i = 0; i < polygon.size(); i++)
      { // Picking an edge that is not parallel to the ray.
         Point2DReadOnly edgeStart = polygon.get(i);
         Point2DReadOnly edgeEnd = ListWrappingIndexTools.getNext(i, polygon);
         Vector2D edgeDirection = new Vector2D();
         edgeDirection.sub(edgeEnd, edgeStart);

         pointOnArbitraryEdge.interpolate(edgeStart, edgeEnd, 0.5);
         rayDirection.sub(pointOnArbitraryEdge, rayOrigin);

         double cross = edgeDirection.cross(rayDirection);

         if (Math.abs(cross) > 1.0e-3)
            break;
      }

      int numberOfIntersections = 0;

      Point2D previousIntersection = null;
      Point2D currentIntersection = null;

      for (int i = 0; i < polygon.size(); i++)
      {
         Point2DReadOnly edgeStart = polygon.get(i);
         Point2DReadOnly edgeEnd = ListWrappingIndexTools.getNext(i, polygon);

         currentIntersection = intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, edgeStart, edgeEnd);

         if (currentIntersection != null)
         { // There is an intersection
            if (previousIntersection == null || !currentIntersection.epsilonEquals(previousIntersection, 1.0e-10))
            { // Because the intersection is different from the previous, the intersection is not on a vertex.
               numberOfIntersections++;
            }
         }

         previousIntersection = currentIntersection;
      }

      if (numberOfIntersections == 0)
      {
         //Could be both outside or inside
         return false;
      }

      // If the number of intersections is odd, the point is inside.
      return numberOfIntersections % 2 != 0;
   }

   public static boolean areBothPointsInsidePolygon(Point2DReadOnly point1, Point2DReadOnly point2, PlanarRegion homeRegion)
   {
      boolean startIsInside = PlanarRegionTools.isPointInLocalInsidePlanarRegion(homeRegion, point1);
      boolean goalIsInside = PlanarRegionTools.isPointInLocalInsidePlanarRegion(homeRegion, point2);

      return startIsInside && goalIsInside;
   }

   public static boolean areBothPointsInsidePolygon(Point2DReadOnly point1, Point2DReadOnly point2, List<? extends Point2DReadOnly> pointsInPolygon)
   {
      Point2D[] pointsArr = pointsInPolygon.toArray(new Point2D[pointsInPolygon.size()]);
      boolean startIsInside = PlanarRegionTools.isPointInsidePolygon(pointsArr, point1);
      boolean goalIsInside = PlanarRegionTools.isPointInsidePolygon(pointsArr, point2);

      return startIsInside && goalIsInside;
   }

   public static boolean isPartOfTheRegionInside(PlanarRegion regionToCheck, PlanarRegion containingRegion)
   {
      ArrayList<Point3D> pointsToCalculateCentroid = new ArrayList<>();
      Point2D[] homePointsArr = new Point2D[containingRegion.getConvexHull().getNumberOfVertices()];
      for (int i = 0; i < containingRegion.getConvexHull().getNumberOfVertices(); i++)
      {
         Point2D point2D = (Point2D) containingRegion.getConvexHull().getVertex(i);
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D fpt = new FramePoint3D();
         fpt.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         containingRegion.getTransformToWorld(transToWorld);
         fpt.applyTransform(transToWorld);
         Point3D transformedPt = fpt.getPoint();

         homePointsArr[i] = new Point2D(transformedPt.getX(), transformedPt.getY());
         pointsToCalculateCentroid.add(new Point3D(transformedPt.getX(), transformedPt.getY(), transformedPt.getZ()));
      }

      ConvexPolygon2D homeConvexPol = new ConvexPolygon2D(homePointsArr);
      homeConvexPol.update();

      Vector3D normal = calculateNormal(containingRegion);

      for (int i = 0; i < regionToCheck.getConvexHull().getNumberOfVertices(); i++)
      {
         Point2D point2D = (Point2D) regionToCheck.getConvexHull().getVertex(i);
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D fpt = new FramePoint3D();
         fpt.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         regionToCheck.getTransformToWorld(transToWorld);
         fpt.applyTransform(transToWorld);

         Point3D pointToProject = fpt.getPoint();
         Point3D projectedPointFromOtherRegion = new Point3D();

         EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point3D, normal, projectedPointFromOtherRegion);

         if (homeConvexPol.isPointInside(new Point2D(projectedPointFromOtherRegion)))
         {
            return true;
         }
      }

      return false;
   }

   public static Vector3D calculateNormal(PlanarRegion region)
   {
      Vector3D normal = new Vector3D();
      region.getNormal(normal);
      return normal;
   }

   public static ArrayList<PlanarRegion> determineWhichRegionsAreInside(PlanarRegion containingRegion, List<PlanarRegion> otherRegionsEx)
   {
      ArrayList<PlanarRegion> regionsInsideHomeRegion = new ArrayList<>();

      for (PlanarRegion otherRegion : otherRegionsEx)
      {
         if (PlanarRegionTools.isPartOfTheRegionInside(otherRegion, containingRegion))
         {
            regionsInsideHomeRegion.add(otherRegion);
         }
      }

      return regionsInsideHomeRegion;
   }

   public static boolean areAllPointsBelowTheRegion(PlanarRegion regionToCheck, PlanarRegion homeRegion)
   {
      for (int i = 0; i < homeRegion.getConcaveHull().length; i++)
      {
         Point2D point2D = (Point2D) homeRegion.getConcaveHull()[i];
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D homeRegionPoint = new FramePoint3D();
         homeRegionPoint.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         homeRegion.getTransformToWorld(transToWorld);
         homeRegionPoint.applyTransform(transToWorld);

         for (int j = 0; j < regionToCheck.getConcaveHull().length; j++)
         {
            Point2D point2D1 = (Point2D) regionToCheck.getConcaveHull()[j];
            Point3D point3D1 = new Point3D(point2D1.getX(), point2D1.getY(), 0);
            FramePoint3D otherRegionPoint = new FramePoint3D();
            otherRegionPoint.set(point3D1);
            RigidBodyTransform transToWorld1 = new RigidBodyTransform();
            regionToCheck.getTransformToWorld(transToWorld1);
            otherRegionPoint.applyTransform(transToWorld1);

            System.out.println(homeRegionPoint.getZ() + "   " + homeRegionPoint.getZ());
            if (homeRegionPoint.getZ() + 0.1 < otherRegionPoint.getZ())
            {
               return false;
            }
         }
      }

      return true;
   }

   public static List<PlanarRegion> ensureClockwiseOrder(List<PlanarRegion> planarRegions)
   {
      List<PlanarRegion> copies = new ArrayList<>(planarRegions.size());

      for (PlanarRegion planarRegion : planarRegions)
      {
         PlanarRegion copy = planarRegion.copy();
         List<Point2DReadOnly> concaveHullVertices = Arrays.asList(copy.getConcaveHull());
         ConcaveHullTools.ensureClockwiseOrdering(concaveHullVertices);
         copies.add(copy);
      }

      return copies;
   }

   public static List<PlanarRegion> filterPlanarRegionsByHullSize(int minNumberOfVertices, List<PlanarRegion> planarRegions)
   {
      if (minNumberOfVertices <= 0)
         return planarRegions;

      return planarRegions.stream().filter(region -> region.getConcaveHull().length >= minNumberOfVertices).collect(Collectors.toList());
   }

   public static List<PlanarRegion> filterPlanarRegionsByArea(double minArea, List<PlanarRegion> planarRegions)
   {
      if (!Double.isFinite(minArea) || minArea <= 0.0)
         return planarRegions;

      return planarRegions.stream().filter(region -> computePlanarRegionArea(region) >= minArea).collect(Collectors.toList());
   }

   public static List<PlanarRegion> filterPlanarRegionsWithBoundingCapsule(Point3DReadOnly capsuleStart, Point3DReadOnly capsuleEnd, double capsuleRadius,
                                                                           List<PlanarRegion> planarRegions)
   {
      return filterPlanarRegionsWithBoundingCapsule(new LineSegment3D(capsuleStart, capsuleEnd), capsuleRadius, planarRegions);
   }

   public static List<PlanarRegion> filterPlanarRegionsWithBoundingCapsule(LineSegment3D capsuleSegment, double capsuleRadius, List<PlanarRegion> planarRegions)
   {
      if (!Double.isFinite(capsuleRadius) || capsuleRadius < 0.0)
         return planarRegions;

      return planarRegions.stream().filter(region -> isPlanarRegionIntersectingWithCapsule(capsuleSegment, capsuleRadius, region)).collect(Collectors.toList());
   }

   public static double computePlanarRegionArea(PlanarRegion planarRegion)
   {
      double area = 0.0;
      for (int i = 0; i < planarRegion.getNumberOfConvexPolygons(); i++)
      {
         area += planarRegion.getConvexPolygon(i).getArea();
      }
      return area;
   }

   public static boolean isPlanarRegionIntersectingWithCapsule(LineSegment3D capsuleSegment, double capsuleRadius, PlanarRegion query)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      query.getTransformToWorld(transformToWorld);

      return Arrays.stream(query.getConcaveHull()).map(vertex -> applyTransform(transformToWorld, vertex))
                   .anyMatch(vertex -> capsuleSegment.distance(vertex) <= capsuleRadius);
   }

   private static Point3D applyTransform(RigidBodyTransform transform, Point2D point2D)
   {
      Point3D point3D = new Point3D(point2D);
      transform.transform(point3D);
      return point3D;
   }

   public static Point3D projectPointToPlane(Point3DReadOnly pointToProject, PlanarRegion regionToProjectTo)
   {
      Vector3D normal = calculateNormal(regionToProjectTo);
      Point2D point2D = (Point2D) regionToProjectTo.getConvexHull().getVertex(0);
      Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);

      return EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point3D, normal);
   }

   public static List<PlanarRegion> keepOnlyRegionsThatAreEntirelyAboveHomeRegion(List<PlanarRegion> regionsToCheck, PlanarRegion homeRegion)
   {
      List<PlanarRegion> filteredList = new ArrayList<>();
      for (PlanarRegion region : regionsToCheck)
      {
         if (isRegionAboveHomeRegion(region, homeRegion))
         {
            filteredList.add(region);
         }
      }

      return filteredList;
   }

   public static boolean isRegionAboveHomeRegion(PlanarRegion regionToCheck, PlanarRegion homeRegion)
   {
      RigidBodyTransform transformFromHomeToWorld = new RigidBodyTransform();
      homeRegion.getTransformToWorld(transformFromHomeToWorld);

      for (int i = 0; i < homeRegion.getConcaveHull().length; i++)
      {
         RigidBodyTransform transformFromOtherToHome = new RigidBodyTransform();
         regionToCheck.getTransformToWorld(transformFromOtherToHome);
         transformFromOtherToHome.preMultiplyInvertOther(transformFromHomeToWorld);

         for (int j = 0; j < regionToCheck.getConcaveHull().length; j++)
         {
            Point3D otherRegionPoint = new Point3D(regionToCheck.getConcaveHull()[j]);
            otherRegionPoint.applyTransform(transformFromOtherToHome);

            if (otherRegionPoint.getZ() < 0.0)
               return false;
         }
      }
      return true;
   }

   public static List<PlanarRegion> filterRegionsByTruncatingVerticesBeneathHomeRegion(List<PlanarRegion> regionsToCheck, PlanarRegion homeRegion,
                                                                                       double depthThresholdForConvexDecomposition, int minTruncatedSize,
                                                                                       double minTruncatedArea)
   {
      List<PlanarRegion> filteredList = new ArrayList<>();
      Point3D pointOnPlane = new Point3D();
      Vector3D planeNormal = new Vector3D();

      homeRegion.getPointInRegion(pointOnPlane);
      homeRegion.getNormal(planeNormal);

      for (PlanarRegion regionToCheck : regionsToCheck)
      {
         PlanarRegion truncatedPlanarRegion = truncatePlanarRegionIfIntersectingWithPlane(pointOnPlane, planeNormal, regionToCheck,
                                                                                          depthThresholdForConvexDecomposition, minTruncatedSize,
                                                                                          minTruncatedArea);
         if (truncatedPlanarRegion != null)
            filteredList.add(truncatedPlanarRegion);
      }

      return filteredList;
   }

   /**
    * Truncate the given planar region {@code planarRegionToTuncate} with the plane such that only
    * the part that is <b>above</b> the plane remains.
    * 
    * @param pointOnPlane a point on the plane. Not modified.
    * @param planeNormal the normal of the plane. Not modified.
    * @param planarRegionToTruncate the original planar region to be truncated. Not modified.
    * @param depthThresholdForConvexDecomposition used to recompute the convex decomposition of the
    *           planar region when it has been truncated.
    * @param minTruncatedSize the minimum number of concave hull vertices for the truncated region
    *           to be created.
    * @param minTruncatedArea the minimum area for the truncated region to be created.
    * @return the truncated planar region which is completely above the plane, or {@code null} if
    *         the given planar region is completely underneath the plane or if it is too small
    *         according to {@code minTruncatedSize} and {@code minTruncatedArea}.
    */
   public static PlanarRegion truncatePlanarRegionIfIntersectingWithPlane(Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal,
                                                                          PlanarRegion planarRegionToTruncate, double depthThresholdForConvexDecomposition,
                                                                          int minTruncatedSize, double minTruncatedArea)
   {
      Point3D pointOnRegion = new Point3D();
      Vector3D regionNormal = new Vector3D();
      planarRegionToTruncate.getPointInRegion(pointOnRegion);
      planarRegionToTruncate.getNormal(regionNormal);

      if (EuclidGeometryTools.areVector3DsParallel(planeNormal, regionNormal, Math.toRadians(3.0)))
      { // The region and the plane are parallel, check which one is above the other.
         double signedDistance = signedDistanceFromPoint3DToPlane3D(pointOnRegion, pointOnPlane, planeNormal);
         if (signedDistance < 0.0)
            return null; // The region is underneath
         else
            return planarRegionToTruncate; // The region is above
      }

      Point3D pointOnPlaneInRegionFrame = new Point3D(pointOnPlane);
      Vector3D planeNormalInRegionFrame = new Vector3D(planeNormal);

      RigidBodyTransform transformFromRegionToWorld = new RigidBodyTransform();
      planarRegionToTruncate.getTransformToWorld(transformFromRegionToWorld);
      pointOnPlaneInRegionFrame.applyInverseTransform(transformFromRegionToWorld);
      planeNormalInRegionFrame.applyInverseTransform(transformFromRegionToWorld);

      Point2DReadOnly vertex2D = planarRegionToTruncate.getConcaveHullVertex(planarRegionToTruncate.getConcaveHullSize() - 1);
      Point3D vertex3D = new Point3D(vertex2D);
      double previousSignedDistance = signedDistanceFromPoint3DToPlane3D(vertex3D, pointOnPlaneInRegionFrame, planeNormalInRegionFrame);

      Point3D previousVertex3D = vertex3D;

      List<Point2D> truncatedConcaveHullVertices = new ArrayList<>();

      boolean isRegionEntirelyAbove = true;
      double epsilonDistance = 1.0e-10;

      for (int i = 0; i < planarRegionToTruncate.getConcaveHullSize(); i++)
      {
         vertex2D = planarRegionToTruncate.getConcaveHullVertex(i);
         vertex3D = new Point3D(vertex2D);

         double signedDistance = signedDistanceFromPoint3DToPlane3D(vertex3D, pointOnPlaneInRegionFrame, planeNormalInRegionFrame);
         isRegionEntirelyAbove &= signedDistance >= -epsilonDistance;

         if (signedDistance * previousSignedDistance < 0.0)
         {
            if (Math.abs(signedDistance) <= epsilonDistance)
            {
               truncatedConcaveHullVertices.add(new Point2D(vertex2D));
            }
            else if (Math.abs(previousSignedDistance) > epsilonDistance)
            {
               Vector3D edgeDirection = new Vector3D();
               edgeDirection.sub(vertex3D, previousVertex3D);
               Point3D intersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlaneInRegionFrame, planeNormalInRegionFrame,
                                                                                                     vertex3D, previousVertex3D);

               truncatedConcaveHullVertices.add(new Point2D(intersection));
            }
         }

         if (signedDistance >= -epsilonDistance)
         {
            truncatedConcaveHullVertices.add(new Point2D(vertex2D));
         }

         previousVertex3D = vertex3D;
         previousSignedDistance = signedDistance;
      }

      if (isRegionEntirelyAbove)
         return planarRegionToTruncate;

      if (truncatedConcaveHullVertices.isEmpty())
         return null; // The region is completely underneath

      if (minTruncatedSize > 0 && truncatedConcaveHullVertices.size() < minTruncatedSize)
         return null; // The resulting region is too small

      List<ConvexPolygon2D> truncatedConvexPolygons = new ArrayList<>();
      ConcaveHullDecomposition.recursiveApproximateDecomposition(new ArrayList<>(truncatedConcaveHullVertices), depthThresholdForConvexDecomposition,
                                                                 truncatedConvexPolygons);

      double totalArea = truncatedConvexPolygons.stream().mapToDouble(ConvexPolygon2D::getArea).sum();
      if (totalArea < minTruncatedArea)
         return null; // The resulting region is too small

      Point2D[] concaveHullVertices = new Point2D[truncatedConcaveHullVertices.size()];
      truncatedConcaveHullVertices.toArray(concaveHullVertices);
      PlanarRegion truncatedRegion = new PlanarRegion(transformFromRegionToWorld, concaveHullVertices, truncatedConvexPolygons);
      truncatedRegion.setRegionId(planarRegionToTruncate.getRegionId());
      return truncatedRegion;
   }

   public static boolean isRegionTooHighToStep(PlanarRegion regionToProject, PlanarRegion regionToProjectTo, double tooHighToStepThreshold)
   {
      Vector3D normal = PlanarRegionTools.calculateNormal(regionToProjectTo);

      for (int i = 0; i < regionToProject.getConvexHull().getNumberOfVertices(); i++)
      {
         Point2D point2D = (Point2D) regionToProject.getConvexHull().getVertex(i);
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D fpt = new FramePoint3D();
         fpt.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         regionToProject.getTransformToWorld(transToWorld);
         fpt.applyTransform(transToWorld);

         Point3D pointToProject = fpt.getPoint();
         Point3D projectedPoint = new Point3D();
         EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point3D, normal, projectedPoint);

         if (pointToProject.distance(projectedPoint) >= tooHighToStepThreshold)
         {
            return true;
         }
      }

      return false;
   }

   public static boolean doRay2DAndLineSegment2DIntersect(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection, Point2DReadOnly lineSegmentStart,
                                                          Point2DReadOnly lineSegmentEnd)
   {
      return doRay2DAndLineSegment2DIntersect(rayOrigin.getX(), rayOrigin.getY(), rayDirection.getX(), rayDirection.getY(), lineSegmentStart.getX(),
                                              lineSegmentStart.getY(), lineSegmentEnd.getX(), lineSegmentEnd.getY());
   }

   public static boolean doRay2DAndLineSegment2DIntersect(double rayOriginX, double rayOriginY, double rayDirectionX, double rayDirectionY,
                                                          double lineSegmentStartX, double lineSegmentStartY, double lineSegmentEndX, double lineSegmentEndY)
   {
      return intersectionBetweenRay2DAndLineSegment2D(rayOriginX, rayOriginY, rayDirectionX, rayDirectionY, lineSegmentStartX, lineSegmentStartY,
                                                      lineSegmentEndX, lineSegmentEndY, null);
   }

   public static Point2D intersectionBetweenRay2DAndLineSegment2D(Point2DReadOnly rayOrigin, Vector2D rayDirection, Point2DReadOnly lineSegmentStart,
                                                                  Point2DReadOnly lineSegmentEnd)
   {
      Point2D intersection = new Point2D();
      boolean success = intersectionBetweenRay2DAndLineSegment2D(rayOrigin.getX(), rayOrigin.getY(), rayDirection.getX(), rayDirection.getY(),
                                                                 lineSegmentStart.getX(), lineSegmentStart.getY(), lineSegmentEnd.getX(), lineSegmentEnd.getY(),
                                                                 intersection);
      if (success)
         return intersection;
      else
         return null;
   }

   public static boolean intersectionBetweenRay2DAndLineSegment2D(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection, Point2DReadOnly lineSegmentStart,
                                                                  Point2DReadOnly lineSegmentEnd, Point2DBasics intersectionToPack)
   {
      return intersectionBetweenRay2DAndLineSegment2D(rayOrigin.getX(), rayOrigin.getY(), rayDirection.getX(), rayDirection.getY(), lineSegmentStart.getX(),
                                                      lineSegmentStart.getY(), lineSegmentEnd.getX(), lineSegmentEnd.getY(), intersectionToPack);
   }

   public static boolean intersectionBetweenRay2DAndLineSegment2D(double rayOriginX, double rayOriginY, double rayDirectionX, double rayDirectionY,
                                                                  double lineSegmentStartX, double lineSegmentStartY, double lineSegmentEndX,
                                                                  double lineSegmentEndY, Point2DBasics intersectionToPack)
   {
      double epsilon = 1.0e-7;

      double lineSegmentDirectionX = lineSegmentEndX - lineSegmentStartX;
      double lineSegmentDirectionY = lineSegmentEndY - lineSegmentStartY;

      double determinant = -rayDirectionX * lineSegmentDirectionY + rayDirectionY * lineSegmentDirectionX;

      double dx = lineSegmentStartX - rayOriginX;
      double dy = lineSegmentStartY - rayOriginY;

      if (Math.abs(determinant) < epsilon)
      { // The ray and line segment are parallel
        // Check if they are collinear
         double cross = dx * rayDirectionY - dy * rayDirectionX;
         if (Math.abs(cross) < epsilon)
         {
            if (EuclidGeometryTools.isPoint2DInFrontOfRay2D(lineSegmentStartX, lineSegmentStartY, rayOriginX, rayOriginY, rayDirectionX, rayDirectionY))
            {
               if (intersectionToPack != null)
                  intersectionToPack.set(lineSegmentStartX, lineSegmentStartY);
               return true;
            }

            if (EuclidGeometryTools.isPoint2DInFrontOfRay2D(lineSegmentEndX, lineSegmentEndY, rayOriginX, rayOriginY, rayDirectionX, rayDirectionY))
            {
               if (intersectionToPack != null)
                  intersectionToPack.set(lineSegmentEndX, lineSegmentEndY);
               return true;
            }

            return false;
         }
         // The ray and line segment are parallel but are not collinear, they do not intersect
         else
         {
            return false;
         }
      }

      double oneOverDeterminant = 1.0 / determinant;
      double AInverse00 = -lineSegmentDirectionY;
      double AInverse01 = lineSegmentDirectionX;
      double AInverse10 = -rayDirectionY;
      double AInverse11 = rayDirectionX;

      double alpha = oneOverDeterminant * (AInverse00 * dx + AInverse01 * dy);
      double beta = oneOverDeterminant * (AInverse10 * dx + AInverse11 * dy);

      boolean areIntersecting = alpha > 0.0 - epsilon && 0.0 - epsilon < beta && beta < 1.0 + epsilon;

      if (areIntersecting && intersectionToPack != null)
      {
         intersectionToPack.setX(rayOriginX + alpha * rayDirectionX);
         intersectionToPack.setY(rayOriginY + alpha * rayDirectionY);
      }

      return areIntersecting;
   }
}
