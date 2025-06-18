import numpy as np
import hppfcl
import pinocchio as pin
import simple
from go2_simulation.abstract_wrapper import AbstractSimulatorWrapper
from go2_simulation.utils import addFloor, addSystemCollisionPairs, setPhysicsProperties, removeBVHModelsIfAny, loadGo2Model

class SimpleSimulator:
    def __init__(self, model, geom_model, visual_model, q0, args):
        self.model = model
        self.geom_model = geom_model
        self.visual_model = visual_model
        self.args = args

        self.data = self.model.createData()
        self.geom_data = self.geom_model.createData()

        for col_req in self.geom_data.collisionRequests:
            col_req: hppfcl.CollisionRequest
            col_req.security_margin = 0.0
            col_req.break_distance = 0.0
            col_req.gjk_tolerance = 1e-6
            col_req.epa_tolerance = 1e-6
            col_req.gjk_initial_guess = hppfcl.GJKInitialGuess.CachedGuess
            col_req.gjk_variant = hppfcl.GJKVariant.DefaultGJK

        for patch_req in self.geom_data.contactPatchRequests:
            patch_req.setPatchTolerance(args["patch_tolerance"])

        # Simulation parameters
        self.simulator = simple.Simulator(model, self.data, geom_model, self.geom_data)
        # admm
        self.simulator.admm_constraint_solver_settings.absolute_precision = args["tol"]
        self.simulator.admm_constraint_solver_settings.relative_precision = args["tol_rel"]
        self.simulator.admm_constraint_solver_settings.max_iter = args["maxit"]
        self.simulator.admm_constraint_solver_settings.mu = args["mu_prox"]
        # pgs 
        self.simulator.pgs_constraint_solver_settings.absolute_precision = args["tol"]
        self.simulator.pgs_constraint_solver_settings.relative_precision = args["tol_rel"]
        self.simulator.pgs_constraint_solver_settings.max_iter = args["maxit"]
        #
        self.simulator.warm_start_constraint_forces = args["warm_start"]
        self.simulator.measure_timings = True
        # Contact patch settings
        self.simulator.constraints_problem.setMaxNumberOfContactsPerCollisionPair(
            args["max_patch_size"]
        )
        # Baumgarte settings
        contact_constraints = self.simulator.constraints_problem.frictional_point_constraint_models
        for i in range(len(contact_constraints)):
            contact_constraints[i].baumgarte_corrector_parameters.Kp = args["Kp"]
            contact_constraints[i].baumgarte_corrector_parameters.Kd = args["Kd"]
        if args["admm_update_rule"] == "spectral":
            self.simulator.admm_constraint_solver_settings.admm_update_rule = (
                pin.ADMMUpdateRule.SPECTRAL
            )
        elif args["admm_update_rule"] == "linear":
            self.simulator.admm_constraint_solver_settings.admm_update_rule = (
                pin.ADMMUpdateRule.LINEAR
            )
        else:
            update_rule = args["admm_update_rule"]
            print(f"ERROR - no match for admm update rule {update_rule}")
            exit(1)
        self.dt = args["dt"]

        # Initialize robot state
        self.q = q0.copy()
        self.v = np.zeros(self.model.nv)
        self.a = np.zeros(self.model.nv)
        self.f_feet = np.zeros(4)

        fps = min([self.args["max_fps"], 1.0 / self.dt])
        self.dt_vis = 1.0 / float(fps)
        self.simulator.reset()


    def execute(self, tau):
        if self.args["contact_solver"] == "ADMM":
            self.simulator.step(self.q, self.v, tau, self.dt)
        else:
            self.simulator.stepPGS(self.q, self.v, tau, self.dt)
        #print(self.simulator.getStepCPUTimes().user)
        self.q = self.simulator.qnew.copy()
        self.v = self.simulator.vnew.copy()
        self.a = self.simulator.anew.copy()

        #print("elapsed simu time " + str(step_end - step_start))
        #time_until_next_step = self.dt_vis - (time.time() - step_start)
        #if time_until_next_step > 0:
        #    time.sleep(time_until_next_step)

        return self.q, self.v, self.a, self.f_feet

    def view_state(self, q):
        self.vizer.display(q)


class SimpleWrapper(AbstractSimulatorWrapper):
    def __init__(self, node, timestep):
        ########################## Load robot model and geometry
        self.rmodel, self.geom_model = loadGo2Model()

        # Load parameters from node
        self.params = {
            'max_fps': node.declare_parameter('max_fps', 30).value,
            'Kp': node.declare_parameter('Kp', 0.0).value,
            'Kd': node.declare_parameter('Kd', 0.0).value,
            'compliance': node.declare_parameter('compliance', 0.0).value,
            'material': node.declare_parameter('material', 'metal').value,
            'horizon': node.declare_parameter('horizon', 1000).value,
            'dt': node.declare_parameter('dt', 1e-3).value,
            'tol': node.declare_parameter('tol', 1e-6).value,
            'tol_rel': node.declare_parameter('tol_rel', 1e-6).value,
            'mu_prox': node.declare_parameter('mu_prox', 1e-4).value,
            'maxit': node.declare_parameter('maxit', 100).value,
            'warm_start': node.declare_parameter('warm_start', 1).value,
            'contact_solver': node.declare_parameter('contact_solver', 'ADMM').value,
            'admm_update_rule': node.declare_parameter('admm_update_rule', 'spectral').value,
            'max_patch_size': node.declare_parameter('max_patch_size', 4).value,
            'patch_tolerance': node.declare_parameter('patch_tolerance', 1e-3).value,
        }

        self.init_simple(timestep)

    def init_simple(self, timestep):
        visual_model = self.geom_model.copy()
        addFloor(self.geom_model, visual_model)

        # Set simulation properties
        self.params["dt"] = timestep
        initial_q = np.array([0, 0, 0.2, 0, 0, 0, 1, 0.0, 1.00, -2.51, 0.0, 1.09, -2.61, 0.2, 1.19, -2.59, -0.2, 1.32, -2.79])
        setPhysicsProperties(self.geom_model, self.params["material"], self.params["compliance"])
        removeBVHModelsIfAny(self.geom_model)
        addSystemCollisionPairs(self.rmodel, self.geom_model, initial_q)

         # Remove all pair of collision which does not concern floor collision
        i = 0
        while i < len(self.geom_model.collisionPairs):
            cp = self.geom_model.collisionPairs[i]
            if self.geom_model.geometryObjects[cp.first].name != 'floor' and self.geom_model.geometryObjects[cp.second].name != 'floor':
                self.geom_model.removeCollisionPair(cp)
            else:
                i = i + 1

        # Create the simulator object
        self.simulator = SimpleSimulator(self.rmodel, self.geom_model, visual_model, initial_q, self.params)

    def step(self, tau_cmd):
        # Execute step and get new state
        torque_simu = np.zeros(self.rmodel.nv)
        torque_simu[6:] = tau_cmd
        q_current, v_current, a_current, f_current = self.simulator.execute(torque_simu)

        return q_current, v_current, a_current, f_current