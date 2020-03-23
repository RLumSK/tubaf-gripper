#!/usr/bin/python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, TU Bergakademie Freiberg
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# author: grehl

import os
from matplotlib.image import NonUniformImage

try:
    from autonomy.PoseGenerator import *
except ImportError as ie:
    import sys
    sys.path.append("/pkg/python/autonomy")
    from PoseGenerator import *


def view_general(generator, show_it=True, print_it=False, ff=['.tex', '.pdf'], save_to="/out/plot", xylim=None):
    """
    Plot all given generators into one figure
    :param save_to: [optional] directory where the plots are stored
    :type save_to: str
    :param ff:  [optional] list of file formats, eg pgf, pdf
    :type ff: list
    :param print_it: [optional] print the plot
    :type print_it: bool
    :param show_it: [optional] show the plot
    :type show_it: bool
    :param generator: pose generator of type PoseGeneratorRosView
    :type generator: PoseGeneratorRosView
    :return: -
    :rtype:-
    """
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.set_xlabel(xlabel="x [m]")
    ax.set_ylabel(ylabel="y [m]")

    if xylim is not None:
        ax.set_xlim(xlim[0], xlim[1])
        ax.set_ylim(*ax.get_xlim())

    name = "Hindernisse"
    ax.plot(generator.obs_points[:, 0], generator.obs_points[:, 1], '.', label=name, zorder=10, color=PoseGeneratorRosView.get_color(name))

    hull = generator.hull_points.tolist()
    hull.append(hull[0])
    as_array = np.asarray(hull)
    name = "Boden"
    ax.plot(as_array[:, 0], as_array[:, 1], '+--', label=name, zorder=9, color=PoseGeneratorRosView.get_color(name))

    # Text
    if generator.subsample > 1.0:
        txt = fig.text(0.15, -0.02, "Teilstichprobe alle " + str(generator.subsample) + " Hindernisse")
    else:
        txt = fig.text(0.15, -0.02,
                       "Teilstichprobe mit " + str(generator.subsample * 100.0) + "% der Hindernisse")

    def legend_without_duplicate_labels(axis):
        handles, labels = axis.get_legend_handles_labels()
        unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
        # Place a legend above this subplot, expanding itself to
        # fully use the given bounding box.
        return axis.legend(*zip(*unique), bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left', ncol=2,
                           mode="expand", borderaxespad=0.)

    lgd = legend_without_duplicate_labels(ax)

    if show_it:
        plt.show()
    if print_it:
        print_plt(file_formats=ff, suffix=u"Szene", save_dir=save_to)
    plt.close(fig)


def view_all(lst_generator, show_it=True, print_it=False, ff=['.tex', '.pdf'], save_to="/out/plot", xylim=[-1, 1], index=-1):
    """
    Plot all given generators into one figure
    :param save_to: [optional] directory where the plots are stored
    :type save_to: str
    :param ff:  [optional] list of file formats, eg pgf, pdf
    :type ff: list
    :param print_it: [optional] print the plot
    :type print_it: bool
    :param show_it: [optional] show the plot
    :type show_it: bool
    :param lst_generator: list of pose generators of type PoseGeneratorRosView
    :type lst_generator: list()
    :param index: scene index
    :type index: int
    :return: -
    :rtype:-
    """
    fig, ax = plt.subplots()
    ax.set_xlabel(xlabel="x [m]")
    ax.set_ylabel(ylabel="y [m]")
    ax.set_xlim(xylim[0], xylim[1])
    ax.set_ylim(*ax.get_xlim())

    for generator in lst_generator:  # type: PoseGeneratorRosView
        if len(generator.obs_points) == 0:
            plt.close()
            return
        generator.plot(ax, hull=False, obstacles=False)
        rospy.loginfo("[view_all(%s)] #obstacles: %g" % (generator.get_name(), len(generator.obs_points)))

    name = "Hindernisse"
    ax.plot(lst_generator[-1].obs_points[:, 0], lst_generator[-1].obs_points[:, 1], '.', label=name, zorder=10, color=PoseGeneratorRosView.get_color(name))

    hull = lst_generator[-1].hull_points.tolist()
    hull.append(hull[0])
    as_array = np.asarray(hull)
    name = "Boden"
    ax.plot(as_array[:, 0], as_array[:, 1], '+--', label=name, zorder=9, color=PoseGeneratorRosView.get_color(name))

    # Text
    if lst_generator[-1].subsample > 1.0:
        txt = fig.text(0.15, -0.02, "Teilstichprobe alle " + str(lst_generator[-1].subsample) + " Hindernisse")
    else:
        txt = fig.text(0.15, -0.02,
                       "Teilstichprobe mit " + str(lst_generator[-1].subsample * 100.0) + "\% der Hindernisse")

    def legend_without_duplicate_labels(axis):
        handles, labels = axis.get_legend_handles_labels()
        unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
        # Place a legend above this subplot, expanding itself to
        # fully use the given bounding box.
        return axis.legend(*zip(*unique), bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left', ncol=2,
                           mode="expand", borderaxespad=0.)

    # lgd = legend_without_duplicate_labels(ax)



    if show_it:
        plt.show()
    if print_it:
        print_plt(file_formats=ff, suffix=u"Errechnete Absetzpunkte - Szene "+str(index), save_dir=save_to)
    plt.close(fig)


def print_plt(file_formats=['.pgf', '.pdf'], extras=[], save_dir="/home/grehl/Schreibtisch/PoseGeneratorImages",
              suffix=''):
    """
    Print the current figure to a file
    :param file_formats:  [optional] list of file formats, eg pgf, pdf
    :type file_formats: list
    :param extras: [optional] passed to matplotlib.savefig(bbox_extra_artists)
    :type extras: list
    :param save_dir: [optional] target directory
    :type save_dir: str
    :param suffix: [optional] file name suffix
    :type suffix: str
    :return: -
    :rtype: -
    """
    if not os.path.exists(save_dir):
        rospy.logwarn("[print_plt] Creating '%s' to store plots" % save_dir)
        os.makedirs(save_dir)

    ts = datetime.datetime.now().strftime("[%Y-%m-%d_%H:%M:%S:%f]")
    for c in file_formats:
        try:
            p = os.path.join(save_dir, c[1:])
            if not os.path.exists(p):
                rospy.logwarn("[print_plt] Creating '%s' to store plots" % p)
                os.makedirs(p)
            p = os.path.join(p, ts + suffix + c)
            if 'tex' in c or 'tikz' in c:
                mpl2tkz.save(p, encoding='utf-8')
            else:
                plt.savefig(p, bbox_extra_artists=extras, bbox_inches='tight')
        except RuntimeError as re:
            rospy.logerr("[print_plt] RuntimeError during format %s\n%s" % (c, re))
        except IndexError as ie:
            rospy.logerr("[print_plt] IndexError during format %s\n%s" % (c, ie))
    plt.close()


def print_tex(generator, save_dir="/home/grehl/Schreibtisch/PoseGeneratorImages"):
    """
    Save the current state in a LaTeX conform plot
    :param generator: pose generator after at least one computation, or alist of them
    :type generator: PoseGeneratorRosView or list
    :param save_dir: directory where the plots are stored
    :type save_dir: str
    :return: -
    :rtype: -
    """
    view_all(generator, show_it=False)
    print_plt(file_formats=[".pdf", ".tex"], save_dir=save_dir)


class EvaluatePoseGenerators(object):
    """
    Evaluation for a PoseGeneratorRosInterface instance
    """

    C_MAP = u'tab20c_r'

    @staticmethod
    def calc_metric_dict(dct_hull, dct_obs, wo=1.0, wh=1.0):
        """
        Calculate a dictionary with the metric based on the hull and obstacle distance
        :param wh: [optional] weight for hull distance
        :type wh: float
        :param wo: [optional] weight for obstacle distance
        :type wo: float
        :param dct_hull: dictionary with hull distances
        :type dct_hull: dict
        :param dct_obs: dictionary with obstacles distances
        :type dct_obs: dict
        :return: dictionary with metric
        :rtype: dict
        """
        dct_metric = {}
        obs_keys = dct_obs.keys()
        for k in dct_hull.keys():
            if k in obs_keys:
                dct_metric[k] = []

        for k in dct_metric:
            dh = dct_hull[k]
            do = dct_obs[k]
            for i in range(0, min(len(do), len(dh))):
                dct_metric[k].append(PoseGeneratorRosInterface.metric(do[i], dh[i], wo, wh))
        return dct_metric

    @staticmethod
    def get_ident(obj):
        return str(type(obj)).split(".")[-1][:-2]

    def __init__(self, generators, timeit=True, save_dir="/out/plot"):
        """
        Default constructor
        :param generators: pose generators
        :type generators: list of PoseGeneratorRosInterface
        :param timeit: measure time during run
        :type timeit: bool
        :param save_dir: directory with plots
        :type save_dir: str
        """
        self._generators = generators

        self.dct_result = {}
        self.dct_lst_hull_distance = {}
        self.dct_lst_obstacle_distance = {}
        self.dct_count_largest_hull_distance = {}
        self.dct_count_largest_obstacle_distance = {}
        self.dct_timing = {}

        self.timeit = timeit
        self.plot_dir = save_dir

        for g in self._generators:  # type: PoseGeneratorRosInterface
            ident = g.get_name()
            self.dct_lst_hull_distance[ident] = []
            self.dct_lst_obstacle_distance[ident] = []
            self.dct_count_largest_hull_distance[ident] = 0
            self.dct_count_largest_obstacle_distance[ident] = 0
            self.dct_timing[ident] = []
            self.dct_result[ident] = []

    def perform(self, timeout=1.0, samples=1000):
        """
        Evaluate the generator for samples
        :param samples: number of iterations
        :type samples: int
        :param timeout: time between triggers
        :type timeout: float
        :return: -
        :rtype: -
        """
        i = 1
        n_samples = samples
        while not rospy.is_shutdown() and i <= n_samples:
            self.run()
            rospy.loginfo("[EvaluatePoseGenerators.perform()] %g/%g" % (i, n_samples))
            rospy.sleep(timeout)
            i += 1
        self.evaluate()

    def run(self, obs=None, flr=None):
        """
        Trigger given generators and save evaluation parameters
        :param obs: [optional] use the given obstacles
        :type obs: MarkerArray
        :param flr: [optional] use the given floor plane
        :type flr: TableArray
        :return: -
        :rtype: -
        """
        while not self._generators[0].check_messages(obs, flr):
            rospy.logdebug("[EvaluatePoseGenerators.run()] Getting new messages")
            obs, flr = self._generators[0].get_messages()
            rospy.sleep(1.0)

        d_hull = 0
        d_obst = d_hull
        hull_ident = ""
        obst_ident = hull_ident

        for g in self._generators:  # type: PoseGeneratorRosInterface
            ident = g.get_name()
            if self.timeit:
                t = time.time()
            g.once(obstacles_msg=obs, floor_msg=flr, current_time=flr.header.stamp)
            if self.timeit:
                self.dct_timing[ident].append(time.time() - t)
            result = g.result
            self.dct_result[ident].append(result)
            obstacles = g.obs_points
            hull = g.hull_points
            if len(hull) == 0 or len(obstacles) == 0:
                return

            hull_distance, _ = PoseGeneratorRosInterface.calc_min_distance(result, hull, mode="PL")
            obstacle_distance, _ = PoseGeneratorRosInterface.calc_min_distance(result, obstacles, mode="PP")
            self.dct_lst_hull_distance[ident].append(hull_distance)
            self.dct_lst_obstacle_distance[ident].append(obstacle_distance)
            rospy.logdebug("[EvaluatePoseGenerators.run() - %s] Min(Distance2Hull) = %g" %
                           (ident, hull_distance))
            rospy.logdebug("[EvaluatePoseGenerators.run() - %s] Min(Distance2Obstacles) = %g" %
                           (ident, obstacle_distance))

            # Evaluate who is best most of the time
            if hull_distance > d_hull:
                d_hull = hull_distance
                hull_ident = ident
            if obstacle_distance > d_obst:
                d_obst = obstacle_distance
                obst_ident = ident

        # Sometimes no obstacles are present, resulting in the same position for all generators,
        # we want to filter such cases
        obs_points, hull_points = self._generators[-1].extract_points(obs, flr.tables[0])
        valid_points = PoseGeneratorRosInterface.in_hull(Delaunay(hull_points[:, :2], qhull_options="Pp"),
                                                         obs_points[:, :2])
        purge_last_evaluation = len(valid_points) == 0
        if purge_last_evaluation:
            rospy.logdebug("[EvaluatePoseGenerators.run()] No obstacles in hull, we wont evaluate this sample")
            for g in self._generators:
                ident = g.get_name()
                if len(self.dct_timing[ident]) > 0:
                    self.dct_timing[ident].pop()
                self.dct_result[ident].pop()
                self.dct_lst_hull_distance[ident].pop()
                self.dct_lst_obstacle_distance[ident].pop()
            return

        self.dct_count_largest_hull_distance[hull_ident] += 1
        self.dct_count_largest_obstacle_distance[obst_ident] += 1

    def evaluate(self, print_it=False, ff=['.tex', '.pdf'], weight_obs=1.0, weight_hull=1.0):
        """
        Plot the gathered data
        :return: -
        :rtype:-
        """
        for k in self.dct_lst_hull_distance.keys():
            if len(self.dct_lst_hull_distance[k]) == 0:
                return

        rospy.loginfo("[EvaluatePoseGenerators.evaluate()] Number of largest distances to the hull:")
        for k in self.dct_count_largest_hull_distance:
            rospy.loginfo("%s\t%g" % (k, self.dct_count_largest_hull_distance[k]))
        rospy.loginfo("[EvaluatePoseGenerators.evaluate()] Number of largest distances to nearest obstacles:")
        for k in self.dct_count_largest_obstacle_distance:
            rospy.loginfo("%s\t%g" % (k, self.dct_count_largest_obstacle_distance[k]))

        n_bin = 25
        alpha = 0.75
        self.plot_hist(self.dct_lst_hull_distance, bins=n_bin, title=u'Abstand zur konvexen Hülle', alpha=alpha)
        if print_it:
            print_plt(file_formats=ff, suffix="hull_histogram", save_dir=self.plot_dir)
        self.plot_hist(self.dct_lst_obstacle_distance, bins=n_bin, title=u'Abstand zum nächsten Hindernis', alpha=alpha)
        if print_it:
            print_plt(file_formats=ff, suffix="obstacle_histogram", save_dir=self.plot_dir)
        dct_metric = EvaluatePoseGenerators.calc_metric_dict(self.dct_lst_hull_distance,
                                                             self.dct_lst_obstacle_distance,
                                                             wo=weight_obs, wh=weight_hull)
        self.plot_hist(dct_metric, bins=n_bin, title=u'Metrik', alpha=alpha)
        if print_it:
            print_plt(file_formats=ff, suffix="metric_histogram", save_dir=self.plot_dir)
        self.plot_hist(self.dct_timing, bins=n_bin, title=u'Rechenzeit', alpha=alpha)
        if print_it:
            print_plt(file_formats=ff, suffix="timing", save_dir=self.plot_dir)

    def distance_to(self, lst_results, n_bin=25, alpha=0.75, print_it=False, show_it=False, ff=['.tex', '.pdf']):
        """
        Determine the distance to a list of results
        :param print_it: whether or not to print the plots
        :type print_it: bool
        :param show_it: whether or not to show_it the plots
        :type show_it: bool
        :param ff: file formats of the print
        :type ff: list
        :param alpha: Alpha channel of the bar plot
        :type alpha: float
        :param lst_results: list of results
        :type lst_results: list
        :param n_bin: number of bins used in bar plot histogram
        :type n_bin: int
        :return: -
        :rtype: -
        """
        dct_distances = {}
        for key in self.dct_result.keys():
            dct_distances[key] = []
        for key in dct_distances.keys():
            for i in range(0, len(lst_results)):
                d = np.linalg.norm(lst_results[i] - self.dct_result[key][i])
                if d != 0:
                    dct_distances[key].append(d)
        for key in self.dct_result.keys():
            if len(dct_distances[key]) == 0:
                del dct_distances[key]
        self.plot_hist(dct_distances, bins=n_bin, title=u'Abstand zur Referenz', alpha=alpha)
        if print_it:
            print_plt(file_formats=ff, save_dir=self.plot_dir, suffix="Referenzevaluierung")
        if show_it:
            plt.show()

    def plot_heatmap(self, name='all', n_bin=20, print_it=False, show_it=False, ff=['.tex', '.pdf']):
        """
        Plot a heatmap of all results
        :param name:
        :type name: str
        :param n_bin: number of bins for the 2d histogram
        :type n_bin: int
        :param print_it: whether or not to print the plots
        :type print_it: bool
        :param show_it: whether or not to show_it the plots
        :type show_it: bool
        :param ff: file formats of the print
        :type ff: list
        :return: -
        :rtype: -
        """
        lst_names = []
        if 'all' in name:
            for g in self._generators:
                lst_names.append(g.get_name())
        else:
            lst_names.append(name)

        rospy.logdebug("[EvaluatePoseGenerators.plot_heatmap()] %s" % lst_names)
        for n in lst_names:
            values = np.asarray(self.dct_result[n])
            if len(values.shape) != 2:
                # TODO: Why do I need this?
                values = np.vstack([values[0], values[1]])
            # See: https://docs.scipy.org/doc/numpy/reference/generated/numpy.histogram2d.html
            data, xedges, yedges = np.histogram2d(values[:, 0].T, values[:, 1].T, bins=n_bin, range=[[-1, 1], [-1, 1]])
            data = data.T
            a_title = "Heatmap " + n
            fig = plt.figure()
            ax = fig.add_subplot(111, title=a_title, aspect='equal', xlim=[-1, 1], ylim=[-1, 1])
            im = NonUniformImage(ax, interpolation='nearest')
            im.set_cmap(EvaluatePoseGenerators.C_MAP)
            im.set_clim(vmin=0, vmax=0.2 * len(values))
            xcenters = (xedges[:-1] + xedges[1:]) / 2
            ycenters = (yedges[:-1] + yedges[1:]) / 2
            rospy.logdebug("[EvaluatePoseGenerators.plot_heatmap(%s)] data:\n %s" % (n, data))
            im.set_data(xcenters, ycenters, data)
            ax.images.append(im)
            plt.colorbar(im)
            if print_it:
                print_plt(file_formats=ff, suffix=a_title, save_dir=self.plot_dir)
            if show_it:
                plt.show()

    @staticmethod
    def plot_hist(dct, **kwargs):
        """
        plot the given dictionary in a histogram
        :param dct: data
        :type dct: dict
        :return: -
        :rtype:-
        """
        my_colors = []
        for k in dct.keys():
            my_colors.append(PoseGeneratorRosView.get_color(k))
        plt.hist(dct.values(), color=my_colors, label=dct.keys())
        # pd.DataFrame(data=dct).plot.hist(color=my_colors, **kwargs)
