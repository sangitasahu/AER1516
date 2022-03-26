import rosbag
import matplotlib.pyplot as plt
import numpy as np
bag = rosbag.Bag('simple.bag')
bbox = np.array([2,2,2])
def signed_dist_pt_plane(pt,n,p):
    return distance = n.dot(pt-p)
def dist_pt_plane(pt,n,p):
    return abs_dist =  np.abs(signed_dist_pt_plane(pt,n,p))


#read_bag file and publish it
#cloud = read_bag()
#cloud.header.frame_id = "map"
#cloud_pub.publish(cloud)
#read path msg
#publish it
#take path and cloud to generate ellipsoid arrays
#take ellipsoid arrays and generate polyhedron arrays
#publish both
for topic, msg, t in bag.read_messages(topics=['/cloud']):
    cloud=msg.points
bag.close()   

p1 = np.array([5,11.5,0.5])
p2 = np.array([13,11.5,3.0])
print(p1,p2)
d = p1+(p2-p1)/2
print(d)


def add_local_bbox(p1,p2,bbox)


# local bounding box around line segment

# plt.scatter(p2[0],p2[1])
# plt.scatter(p1[0],p1[1])
# plt.scatter(d[0],d[1])



# plt.show()



    Ellipsoid<Dim> E(C, (p1_ + p2_) / 2);
    auto Rf = Ri;

    ////// Let's inflate the obstacles now: Substitute all the points in obs_ by the nearest vertex of the cube centered
    /// at that point and paralell to the axis of the ellipsoid
    for (auto &it : this->obs_)
    {
      // std::cout << "Inflating the obstacles!!" << std::endl;
      Vecf<Dim> p = Ri.transpose() * (it - E.d());  // To Ellipsoid frame
      Vecf<Dim> tmp;                                // New Point in Ellipsoid frame

      tmp(0) = p(0) - sgn(p(0)) * inflate_distance_;
      tmp(1) = p(1) - sgn(p(1)) * inflate_distance_;
      tmp(2) = p(2) - sgn(p(2)) * inflate_distance_;
      it = Ri * tmp + E.d();  // Substitute previous point by the new Point in World frame
    }
    ///// Obstacles inflated

    auto obs = E.points_inside(this->obs_);
    auto obs_inside = obs;
    //**** decide short axes
    while (!obs_inside.empty())
    {
      const auto pw = E.closest_point(obs_inside);
      Vecf<Dim> p = Ri.transpose() * (pw - E.d());  // to ellipsoid frame
      const decimal_t roll = atan2(p(2), p(1));
      Rf = Ri * Quatf(cos(roll / 2), sin(roll / 2), 0, 0);
      p = Rf.transpose() * (pw - E.d());

      if (p(0) < axes(0))
        axes(1) = std::abs(p(1)) / std::sqrt(1 - std::pow(p(0) / axes(0), 2));
      Matf<Dim, Dim> new_C = Matf<Dim, Dim>::Identity();
      new_C(0, 0) = axes(0);
      new_C(1, 1) = axes(1);
      new_C(2, 2) = axes(1);
      E.C_ = Rf * new_C * Rf.transpose();

      vec_Vecf<Dim> obs_new;
      for (const auto &it : obs_inside)
      {
        if (1 - E.dist(it) > epsilon_)
          obs_new.push_back(it);
      }
      obs_inside = obs_new;
    }

    //**** reset ellipsoid with old axes(2)
    C = f * Matf<Dim, Dim>::Identity();
    C(0, 0) = axes(0);
    C(1, 1) = axes(1);
    C(2, 2) = axes(2);
    E.C_ = Rf * C * Rf.transpose();
    obs_inside = E.points_inside(obs);

    while (!obs_inside.empty())
    {
      const auto pw = E.closest_point(obs_inside);
      Vec3f p = Rf.transpose() * (pw - E.d());
      decimal_t dd = 1 - std::pow(p(0) / axes(0), 2) - std::pow(p(1) / axes(1), 2);
      if (dd > epsilon_)
        axes(2) = std::abs(p(2)) / std::sqrt(dd);
      Matf<Dim, Dim> new_C = Matf<Dim, Dim>::Identity();
      new_C(0, 0) = axes(0);
      new_C(1, 1) = axes(1);
      new_C(2, 2) = axes(2);
      E.C_ = Rf * new_C * Rf.transpose();

      vec_Vecf<Dim> obs_new;
      for (const auto &it : obs_inside)
      {
        if (1 - E.dist(it) > epsilon_)
          obs_new.push_back(it);
      }
      obs_inside = obs_new;
    }

    this->ellipsoid_ = E;
  }

