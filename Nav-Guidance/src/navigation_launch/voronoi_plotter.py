from matplotlib.collections import LineCollection
import matplotlib.pyplot as ax
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon



def scale_down(shift_size,x1,y1,x2,y2):
    return (x1/shift_size,y1/shift_size,x2/shift_size,y2/shift_size)

def point_distance(vec1, vec2):
    """distance of 2 vec2d objects"""
    shift_size = 1000
    x1, y1, x2, y2 = scale_down(shift_size,vec1.x,vec1.y,vec2.x,vec2.y)#need to scale down or exponential overflows
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return dist*shift_size

def voronoi(box_vertices):
    """takes in box vertices as Vec2d objects should return the voronoi vertices as Vec2d objects"""
    list_box_cloud = Vec2d_to_list(box_vertices)#voronoi function takes in np array of lists not Vec2d
    plot = np.array(list_box_cloud)
    vor = Voronoi(plot)
    voronoi_vecs=[]
    for vertex in vor.vertices:
        voronoi_vecs.append(Vec2d.from_point(vertex[0],vertex[1]))
    return voronoi_vecs

def get_traversable_vertices(vor):
    """takes in voronoi diagram and returns np array of traversable vertices as Vec2d objects"""
    finite_segments = []
    for pointidx, simplex in zip(vor.ridge_points,
                                 vor.ridge_vertices):  # find the traversable segments (from voronoi_plot_2d)
        simplex = np.asarray(simplex)
        if np.all(simplex >= 0):
            finite_segments.append(vor.vertices[simplex])

    traversable_vertices = []
    for segment in finite_segments:
        seg_start = Vec2d.from_point(segment[0][0], segment[0][1])  # convert nparray cartesian back to vec2d
        seg_end = Vec2d.from_point(segment[1][0], segment[1][1])  # same conversion here
        traversable_vertices.append([seg_start, seg_end])
    return traversable_vertices

def flatten_contours(pointCloud):
    """merges contours of Vec2d objects"""
    cloud = []
    contour_count=0
    for contour in pointCloud:
        contour_count+=1
        for point in contour:
            cloud.append(point)
    return cloud

def group_as_rectangles(depth_map, min_cluster_size, point_spacing):
    """
    returns vec2d objects that represent corners of rectangle objects these will need to be flattened before finding
    the voronoi diagram.
    """

    clusters=[]
    vec_iterator = iter(depth_map)

    try:#first make clusters
        vec1 = next(vec_iterator)
        cluster=[]
        while True:
            vec2 = next(vec_iterator)
            dist=point_distance(vec1,vec2)
            if dist < point_spacing:#add to current cluster
                cluster.append(vec1)
            else: #outside of current cluster
                if len(cluster)> min_cluster_size:#legitimize cluster if it has enough points
                    clusters.append(cluster)#add cluster to clusters
                cluster=[]#reset cluster
            vec1=vec2 #shift up
    except StopIteration:
        pass

    rectangles = []#now package clusters in rectangles (predictable geometry)
    for cluster in clusters:
        cluster.sort(key=lambda x: x.x)
        length=len(cluster)
        xmin=cluster[0].x
        xmax=cluster[length-1].x
        cluster.sort(key=lambda x: x.y)
        ymin=cluster[0].y
        ymax=cluster[length-1].y
        rectangles.append([Vec2d.from_point(xmin,ymin),Vec2d.from_point(xmin,ymax),Vec2d.from_point(xmax,ymax),Vec2d.from_point(xmax,ymin)])

    return rectangles





def main():
    clusters = group_as_rectangles(depth_map=lidar, min_cluster_size=3, point_spacing=500)
    box_cloud = flatten_contours(clusters)  # must be 1d array of points
    vor = Voronoi(np.array(Vec2d_to_list(box_cloud)))  # get voronoi diagram properties
    traversable_vertices = get_traversable_vertices(vor)#get np array of segments we can traverse

    #filter out the segments that are inside polygons(obstacles)
    # doesnot work 100%, still trying to debug.
    for line in traversable_vertices:
        for cluster in clusters:
            point1 = Point(line[0].x, line[0].y)
            point2 = Point(line[1].x, line[1].y)
            polygon = Polygon([(cluster[0].x, cluster[0].y), (cluster[1].x, cluster[1].y), (cluster[2].x, cluster[2].y),
                               (cluster[3].x, cluster[3].y)])
            if polygon.contains(point1) or polygon.contains(point2):
                traversable_vertices.remove(line)


