import roslaunch
import rospy

import node_proxy


def get_resolved_filepath(package, launchfile):
    args = package, launchfile
    return roslaunch.rlutil.resolve_launch_arguments(args)[0]


def get_formatted_launch_args(launch_args):
    return ['%s:=%s' % (key,val) for key, val in launch_args.items()]


class LaunchParent:
    def __init__(self, package, launchfile, args={}):
        self.run_id = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.name = '%s/%s' % (package, launchfile)
        self.package = package
        self.launchfile = launchfile
        self.args = args
        self.config = None
        self.runner = None
        self.nodes = None

    def initiate_launches(self):
        rospy.loginfo('initiating [%s] launches', self.name)
        self.build_launch_config()
        self.create_node_procs()
        self.load_launch_parameters()
        rospy.loginfo('[%s] launches complete', self.name)

    def destroy_launches(self):
        rospy.loginfo('destroying [%s] launches', self.name)
        self.runner = None
        self.destroy_node_procs()
        self.config = None
        rospy.loginfo('[%s] launches destroyed', self.name)

    def build_launch_config(self, verbose=False):
        loader = roslaunch.xmlloader.XmlLoader()
        config = roslaunch.config.ROSLaunchConfig()
        launch_args = get_formatted_launch_args(self.args)
        filepath = get_resolved_filepath(self.package, self.launchfile)
        loader.load(filepath, config, argv=launch_args, verbose=verbose)
        self.config = config

    def create_node_procs(self):
        nodes = {}
        for resolved_name, node in zip(self.config.resolved_node_names,
                                       self.config.nodes):
            try:
                proxy = node_proxy.NodeProxy(node)
                proxy.create_node_process(self.run_id, self.config)
                nodes[resolved_name] = proxy
                rospy.loginfo('- [%s] created', resolved_name)
            except node_proxy.NodeProxyException as e:
                rospy.logerr(e)
        self.nodes = nodes

    def destroy_node_procs(self):
        if self.nodes is not None:
            for nodename in self.nodes:
                self.stop_node(nodename)
                self.remove_node(nodename)
                rospy.loginfo('- [%s] removed', nodename)

    def load_launch_parameters(self):
        self.runner = roslaunch.launch.ROSLaunchRunner(self.run_id, self.config)
        self.runner._load_parameters()

    def get_node(self, nodename):
        node = self.nodes.get(nodename)
        if node is None:
            rospy.logwarn('node [%s] has not been loaded', nodename)
        return node

    def start_node(self, nodename):
        node = self.get_node(nodename)
        node.start_process()

    def stop_node(self, nodename):
        node = self.get_node(nodename)
        node.stop_process()

    def start_all(self):
        if self.nodes is None:
            rospy.logwarn('node processes have not been initiated')
            return
        for nodename in self.nodes:
            self.start_node(nodename)

    def stop_all(self):
        if self.nodes is None:
            return
        for nodename in self.nodes:
            self.stop_node(nodename)

    def remove_node(self, nodename):
        try:
            self.nodes[nodename] = None
        except KeyError:
            pass

    def __del__(self):
        self.destroy_launches()
