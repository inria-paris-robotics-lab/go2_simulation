import numpy as np
from go2_description.loader import loadGo2
import hppfcl
import pinocchio as pin
import simple
from go2_simulation.abstract_wrapper import AbstractSimulatorWrapper

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
        self.foot_names = ["FR_foot_0", "FL_foot_0", "RR_foot_0", "RL_foot_0"]
        self.all_col_pairs = self.simulator.geom_model.collisionPairs.tolist()

        fps = min([self.args["max_fps"], 1.0 / self.dt])
        self.dt_vis = 1.0 / float(fps)
        self.simulator.reset()


    def execute(self, tau):
        if self.args["contact_solver"] == "ADMM":
            self.simulator.step(self.q, self.v, tau, self.dt)
        else:
            self.simulator.stepPGS(self.q, self.v, tau, self.dt)

        self.q = self.simulator.qnew.copy()
        self.v = self.simulator.vnew.copy()
        self.a = self.simulator.anew.copy()
        
        # Detect contact through pair of collision
        current_col_pairs = self.simulator.constraints_problem.pairs_in_collision.tolist()
        self.f_feet = np.zeros(4)
        for cp_id in current_col_pairs:
            cp = self.all_col_pairs[cp_id]
            first = self.simulator.geom_model.geometryObjects[cp.first].name
            second = self.simulator.geom_model.geometryObjects[cp.second].name
            if (first in self.foot_names) or (second in self.foot_names):
                self.f_feet[self.foot_names.index(first)] = 1

        return self.q, self.v, self.a, self.f_feet


def setPhysicsProperties(
    geom_model: pin.GeometryModel, material: str, compliance: float
):
    for gobj in geom_model.geometryObjects:
        if material == "ice":
            gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.ICE
        elif material == "plastic":
            gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.PLASTIC
        elif material == "wood":
            gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.WOOD
        elif material == "metal":
            gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.METAL
        elif material == "concrete":
            gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.CONCRETE

        # Compliance
        gobj.physicsMaterial.compliance = compliance


def removeBVHModelsIfAny(geom_model: pin.GeometryModel):
    for gobj in geom_model.geometryObjects:
        gobj: pin.GeometryObject
        bvh_types = [hppfcl.BV_OBBRSS, hppfcl.BV_OBB, hppfcl.BV_AABB]
        ntype = gobj.geometry.getNodeType()
        if ntype in bvh_types:
            gobj.geometry.buildConvexHull(True, "Qt")
            gobj.geometry = gobj.geometry.convex


def addFloor(geom_model: pin.GeometryModel, visual_model: pin.GeometryModel):   
    # Collision object
    floor_collision_shape = hppfcl.Halfspace(0, 0, 1, 0)
    M = pin.SE3.Identity()
    floor_collision_object = pin.GeometryObject("floor", 0, 0, M, floor_collision_shape)
    geom_model.addGeometryObject(floor_collision_object)

def addSystemCollisionPairs(model, geom_model, qref):
    """
    Add the right collision pairs of a model, given qref.
    qref is here as a `T-pose`. The function uses this pose to determine which objects are in collision
    in this ref pose. If objects are in collision, they are not added as collision pairs, as they are considered
    to always be in collision.
    """
    data = model.createData()
    geom_data = geom_model.createData()
    pin.updateGeometryPlacements(model, data, geom_model, geom_data, qref)
    geom_model.removeAllCollisionPairs()
    num_col_pairs = 0
    for i in range(len(geom_model.geometryObjects)):
        for j in range(i+1, len(geom_model.geometryObjects)):
            # Don't add collision pair if same object
            if i != j:
                gobj_i: pin.GeometryObject = geom_model.geometryObjects[i]
                gobj_j: pin.GeometryObject = geom_model.geometryObjects[j]
                if gobj_i.name == "floor" or gobj_j.name == "floor":
                    num_col_pairs += 1
                    col_pair = pin.CollisionPair(i, j)
                    geom_model.addCollisionPair(col_pair)
                else:
                    if gobj_i.parentJoint != gobj_j.parentJoint or gobj_i.parentJoint == 0:
                        if gobj_i.parentJoint != model.parents[gobj_j.parentJoint] and gobj_j.parentJoint != model.parents[gobj_i.parentJoint] or gobj_i.parentJoint == 0 or gobj_j.parentJoint == 0:
                            # Compute collision between the geometries. Only add the collision pair if there is no collision.
                            M1 = geom_data.oMg[i]
                            M2 = geom_data.oMg[j]
                            colreq = hppfcl.CollisionRequest()
                            colreq.security_margin = 1e-2 # 1cm of clearance
                            colres = hppfcl.CollisionResult()
                            hppfcl.collide(gobj_i.geometry, M1, gobj_j.geometry, M2, colreq, colres)
                            if not colres.isCollision():
                                num_col_pairs += 1
                                col_pair = pin.CollisionPair(i, j)
                                geom_model.addCollisionPair(col_pair)
    print("Num col pairs = ", num_col_pairs)
    return num_col_pairs


class SimpleWrapper(AbstractSimulatorWrapper):
    def __init__(self, node, timestep):
        ########################## Load robot model and geometry
        robot = loadGo2()
        self.rmodel = robot.model
        self.geom_model = robot.collision_model
        self.visual_model = robot.visual_model

        # Ignore friction and kinematics limits inside the simulator
        for i in range(self.rmodel.nq):
            self.rmodel.lowerPositionLimit[i] = np.finfo("d").min
            self.rmodel.upperPositionLimit[i] = np.finfo("d").max 
        self.rmodel.lowerDryFrictionLimit[:] = 0
        self.rmodel.upperDryFrictionLimit[:] = 0

        # Load parameters from node
        self.params = {
            'max_fps': node.declare_parameter('max_fps', 30).value,
            'Kp': node.declare_parameter('Kp', 0.0).value,
            'Kd': node.declare_parameter('Kd', 0.0).value,
            'compliance': node.declare_parameter('compliance', 0.0).value,
            'material': node.declare_parameter('material', 'metal').value,
            'horizon': node.declare_parameter('horizon', 1000).value,
            'dt': node.declare_parameter('dt', timestep).value,
            'tol': node.declare_parameter('tol', 1e-6).value,
            'tol_rel': node.declare_parameter('tol_rel', 1e-6).value,
            'mu_prox': node.declare_parameter('mu_prox', 1e-4).value,
            'maxit': node.declare_parameter('maxit', 100).value,
            'warm_start': node.declare_parameter('warm_start', 1).value,
            'contact_solver': node.declare_parameter('contact_solver', 'PGS').value,
            'admm_update_rule': node.declare_parameter('admm_update_rule', 'spectral').value,
            'max_patch_size': node.declare_parameter('max_patch_size', 2).value,
            'patch_tolerance': node.declare_parameter('patch_tolerance', 1e-2).value,
        }

        self.init_simple()

    def init_simple(self):
        # Start the robot in crouch pose 15cm above the ground
        initial_q = np.array([0, 0, 0.15, 0, 0, 0, 1, 0.0, 0.9, -2.5, 0.0, 0.9, -2.5, 0., 0.9, -2.5, 0, 0.9, -2.5])

        # Set simulation properties
        addFloor(self.geom_model, self.visual_model)
        setPhysicsProperties(self.geom_model, self.params["material"], self.params["compliance"])
        removeBVHModelsIfAny(self.geom_model)
        addSystemCollisionPairs(self.rmodel, self.geom_model, initial_q)

        # Unitree joint ordering (FR, FL, RR, RL)
        self.joint_order = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]

        # Create the simulator object
        self.simulator = SimpleSimulator(self.rmodel, self.geom_model, self.visual_model, initial_q, self.params)

    def step(self, tau_cmd):
        # Change torque order from unitree to pinocchio
        torque_simu = np.zeros(self.rmodel.nv)
        for i in range(12):
            torque_simu[6 + i] = tau_cmd[self.joint_order[i]]

        # Execute step and get new state
        q_current, v_current, a_current, f_current = self.simulator.execute(torque_simu)

        # Reorder state from pinocchio to unitree order
        q_unitree = q_current.copy()
        v_unitree = v_current.copy()
        a_unitree = a_current.copy()
        for i in range(12):
            q_unitree[7 + i] = q_current[7 + self.joint_order[i]]
            v_unitree[6 + i] = v_current[6 + self.joint_order[i]]
            a_unitree[6 + i] = a_current[6 + self.joint_order[i]]
        
        # Reorder contacts from (FL, FR, RR, RL) to (FR, FL, RR, RL)
        f_unitree = np.array([f_current[1], f_current[0], f_current[3], f_current[2]])

        return q_unitree, v_unitree, a_unitree, f_unitree