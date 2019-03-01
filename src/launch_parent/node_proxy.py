import rosgraph
import roslaunch
import rospy

from threading import Lock

STATUS_IDLE    = 'idle'
STATUS_STARTED = 'started'
STATUS_EXITED  = 'exited'


class NodeProxyException(Exception):
    pass


class NodeProxy:
    def __init__(self, node):
        self.node = node
        self._proc = None

    def status(self):
        status = STATUS_IDLE
        if self._proc is None:
            return status
        if self._proc.started:
            status = STATUS_STARTED
        if not self._proc.is_alive():
            status = STATUS_EXITED
        return status

    def is_idle(self):
        return self.status() == STATUS_IDLE

    def is_started(self):
        return self.status() == STATUS_STARTED

    def is_exited(self):
        return self.status() == STATUS_EXITED

    def create_node_process(self, run_id, config):
        rospy.logdebug('creating node process [%s/%s]',
                       self.node.package,
                       self.node.type)

        if self.node.machine is None:
            self.node.machine = config.machines['']
        if self.node.name is None:
            self.node.name = rosgraph.names.anonymous_name(node.type)

        try:
            self._proc = create_node_process(
                run_id, self.node, config.master.uri)
        except roslaunch.node_args.NodeParamsException as e:
            raise NodeProxyException(
                'failed to create node process of type [%s/%s]' % (self.node.package,
                                                                   self.node.type))

    def start_process(self):
        try:
            status = self.status()
            if status == STATUS_STARTED:
                return
            rospy.loginfo('starting node process [%s/%s]',
                          self.node.package,
                          self.node.type)
            self._proc.start()
        except Exception as e:
            rospy.logerr('error starting node process [%s/%s]: %s',
                          self.node.package,
                          self.node.type,
                          e)

    def stop_process(self):
        try:
            status = self.status()
            if status == STATUS_EXITED or status == STATUS_IDLE:
                return
            rospy.loginfo('stopping node process [%s/%s]',
                          self.node.package,
                          self.node.type)
            self._proc.stop()
        except Exception as e:
            rospy.logerr('error stopping node process [%s/%s]: %s',
                         self.node.package,
                         self.node.type,
                         e)

    def get_proc_name(self):
        return self._proc.name

    def get_exit_code(self):
        return self._proc.exit_code

    def get_time_of_death(self):
        return self._proc.time_of_death

    def to_dict(self):
        return {
            'name': self.node.name,
            'namespace': self.node.namespace,
            'package': self.node.package,
            'status': self.status(),
            'type': self.node.type,
            'proc_exit_description': self._proc.get_exit_description(),
            'proc_name': self.get_proc_name(),
            'proc_pid': self._proc.pid,
            'proc_time_of_death': self.get_time_of_death(),
        }

    def __repr__(self):
        template = '<NodeProxy pkg={} type={}>'
        return template.format(self.node.package, self.node.type)

    def __del__(self):
        self.stop_process()
        self._proc = None


def create_node_process(run_id, node, master_uri):
    env = roslaunch.nodeprocess.setup_env(node, node.machine, master_uri)
    name = "%s-%s" % (rosgraph.names.ns_join(node.namespace, node.name),
                      roslaunch.nodeprocess._next_counter())
    if name[0] == '/': 
        name = name[1:]
    args = roslaunch.nodeprocess.create_local_process_args(node, node.machine)
    log_output = node.output != 'screen'
    return LocalProcess(
        run_id, node.package, name, args, env, log_output,
        respawn=node.respawn, respawn_delay=node.respawn_delay,
        required=node.required, cwd=node.cwd)


class LocalProcess(roslaunch.nodeprocess.LocalProcess):
    """
    Subclassed to avoid problematic initialization behavior
    """
    
    def __init__(self, run_id, package, name, args, env, log_output,
            respawn=False, respawn_delay=0.0, required=False, cwd=None,
            is_node=True):
        """
        @param run_id: unique run ID for this roslaunch. Used to
          generate log directory location. run_id may be None if this
          feature is not being used.
        @type  run_id: str
        @param package: name of package process is part of
        @type  package: str
        @param name: name of process
        @type  name: str
        @param args: list of arguments to process
        @type  args: [str]
        @param env: environment dictionary for process
        @type  env: {str : str}
        @param log_output: if True, log output streams of process
        @type  log_output: bool
        @param respawn: respawn process if it dies (default is False)
        @type  respawn: bool
        @param respawn_delay: respawn process after a delay
        @type  respawn_delay: float
        @param cwd: working directory of process, or None
        @type  cwd: str
        @param is_node: (optional) if True, process is ROS node and accepts ROS node command-line arguments. Default: True
        @type  is_node: False
        """    
        self.package = package
        self.name = name
        self.args = args
        self.env = env
        self.respawn = respawn
        self.respawn_delay = respawn_delay
        self.required = required
        self.lock = Lock()
        self.exit_code = None
        self.spawn_count = 0
        self.time_of_death = None
        self.run_id = run_id
        self.popen = None
        self.log_output = log_output
        self.started = False
        self.stopped = False
        self.cwd = cwd
        self.log_dir = None
        self.pid = -1
        self.is_node = is_node
