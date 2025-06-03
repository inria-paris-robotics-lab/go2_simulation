import numpy as np
from go2_description import loadGo2
from go2_description import GO2_DESCRIPTION_URDF_PATH, GO2_DESCRIPTION_PACKAGE_DIR
import hppfcl
import pinocchio as pin
import simple

class SimulationArgs():
    num_repetitions: int = 1
    display: bool = False
    display_com: bool = False
    debug: bool = False
    debug_step: int = 1
    display_collision_model: bool = True
    display_step: bool = False
    display_state: bool = False
    display_contacts: bool = True
    debug_transparency: float = 0.5
    max_fps: int = 30
    Kp: float = 0 # baumgarte proportional term
    Kd: float = 0 # baumgarte derivative term
    compliance: float = 0.0
    material: str = "metal"
    horizon: int = 1000
    dt: float = 1e-3
    tol: float = 1e-3
    tol_rel: float = 1e-3
    mu_prox: float = 1e-2
    maxit: int = 100
    warm_start: int = 1
    contact_solver: str = "PGS"
    plot_metrics: bool = False
    plot_hist: bool = False
    plot_title: str = "NO TITLE"
    seed: int = 1234
    random_initial_velocity: bool = False
    add_damping: bool = False
    damping_factor: float = 0.0
    admm_update_rule: str = "spectral"
    mujoco_show_ui: bool = False
    max_patch_size: int = 4
    patch_tolerance: float = 1e-2

    def process_args(self):
        if self.debug:
            self.display = True
            self.display_contacts = True
            self.display_state = True
            self.display_com = True

class Simulation:
    def __init__(self, model, geom_model, visual_model, q0, v0, args):
        self.model = model
        self.geom_model = geom_model
        self.visual_model = visual_model
        self.q0 = q0
        self.v0 = v0
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
            patch_req.setPatchTolerance(args.patch_tolerance)

        # Simulation parameters
        self.simulator = simple.SimulatorInstance(model, self.data, geom_model, self.geom_data)
        self.simulator.contact_solver_info = pin.ProximalSettings(
            args.tol, args.tol_rel, args.mu_prox, args.maxit
        )
        self.simulator.warm_start_constraint_forces = args.warm_start
        self.simulator.measure_timings = True
        # Contact patch settings
        self.simulator.constraints_problem.setMaxNumberOfContactsPerCollisionPair(
            args.max_patch_size
        )
        # Baumgarte settings
        self.simulator.constraints_problem.Kp = args.Kp
        self.simulator.constraints_problem.Kd = args.Kd
        if args.admm_update_rule == "spectral":
            self.simulator.admm_constraint_solver_settings.admm_update_rule = (
                pin.ADMMUpdateRule.SPECTRAL
            )
        elif args.admm_update_rule == "linear":
            self.simulator.admm_constraint_solver_settings.admm_update_rule = (
                pin.ADMMUpdateRule.LINEAR
            )
        else:
            print(f"ERROR - no match for admm update rule {args.admm_update_rule}")
            exit(1)
        self.dt = args.dt
        self.q = q0.copy()
        self.v = v0.copy()
        self.fext = [pin.Force(np.zeros(6)) for _ in range(model.njoints)]
        fps = min([self.args.max_fps, 1.0 / self.dt])
        self.dt_vis = 1.0 / float(fps)
        self.simulator.reset()

        pass

    def execute(self, tau):
        if self.args.contact_solver == "ADMM":
            self.simulator.step(self.q, self.v, tau, self.fext, self.dt)
        else:
            self.simulator.stepPGS(self.q, self.v, tau, self.fext, self.dt)
        #print(self.simulator.getStepCPUTimes().user)
        self.q = self.simulator.qnew.copy()
        self.v = self.simulator.vnew.copy()

        #print("elapsed simu time " + str(step_end - step_start))
        #time_until_next_step = self.dt_vis - (time.time() - step_start)
        #if time_until_next_step > 0:
        #    time.sleep(time_until_next_step)
    
    def get_state(self):
        return self.q, self.v


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
    # floor_collision_shape = hppfcl.Box(10, 10, 2)
    # M = pin.SE3(np.eye(3), np.zeros(3))
    # M.translation = np.array([0.0, 0.0, -(1.99 / 2.0)])
    floor_collision_shape = hppfcl.Halfspace(0, 0, 1, 0)
    # floor_collision_shape = hppfcl.Plane(0, 0, 1, 0)
    floor_collision_shape.setSweptSphereRadius(0.1)
    M = pin.SE3.Identity()
    floor_collision_object = pin.GeometryObject("floor", 0, 0, M, floor_collision_shape)
    geom_model.addGeometryObject(floor_collision_object)

    # Visual object
    floor_visual_shape = hppfcl.Box(10, 10, 0.01)
    floor_visual_object = pin.GeometryObject(
        "floor", 0, 0, pin.SE3.Identity(), floor_visual_shape
    )
    visual_model.addGeometryObject(floor_visual_object)
        
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

class SimpleWrapper():
    def __init__(self, node, timestep):
        ########################## Load robot model and geometry
        robot = loadGo2()
        self.rmodel = robot.model
        self.q0 = self.rmodel.referenceConfigurations["standing"]
        self.njoints = self.rmodel.nv - 6

        with open(GO2_DESCRIPTION_URDF_PATH, 'r') as file:
            file_content = file.read()

        #self.geom_model = pin.GeometryModel()
        #pin.buildGeomFromUrdfString(self.rmodel, file_content, pin.GeometryType.VISUAL, self.geom_model, GO2_DESCRIPTION_PACKAGE_DIR)
        self.geom_model = robot.collision_model
        self.visual_model = robot.visual_model

        self.init_simple(timestep)

    def init_simple(self, timestep):
        addFloor(self.geom_model, self.visual_model)
        args = SimulationArgs()

        # Set simulation properties
        args.dt = timestep
        initial_q = np.array([0, 0, 0.5, 0, 0, 0, 1, 0.0, 1.00, -2.51, 0.0, 1.09, -2.61, 0.2, 1.19, -2.59, -0.2, 1.32, -2.79])
        #initial_q = np.array([0, 0, 0.5, 0, 0, 0, 1, 0.0, 0.78, -1.44, 0, 0.78, -1.44, 0, 0.78, -1.44, 0, 0.78, -1.44])
        setPhysicsProperties(self.geom_model, args.material, args.compliance)
        removeBVHModelsIfAny(self.geom_model)
        addSystemCollisionPairs(self.rmodel, self.geom_model, self.q0)

        # Remove all pair of collision which does not concern floor collision
        i = 0
        while i < len(self.geom_model.collisionPairs):
            cp = self.geom_model.collisionPairs[i]
            if self.geom_model.geometryObjects[cp.first].name != 'floor' and self.geom_model.geometryObjects[cp.second].name != 'floor':
                self.geom_model.removeCollisionPair(cp)
            else:
                i = i + 1

        # Create the simulator object
        self.simulator = Simulation(self.rmodel, self.geom_model, self.visual_model, initial_q, np.zeros(self.rmodel.nv), args) 

    def get_state(self):
        q_current, v_current = self.simulator.get_state()

        return q_current, v_current

    def execute_step(self, tau_des, q_des, v_des, kp_des, kd_des):
        # Get sub step state
        q_current, v_current = self.simulator.get_state()
        tau_cmd = tau_des - np.multiply(q_current[7:]-q_des, kp_des) - np.multiply(v_current[6:]-v_des, kd_des)

        # Set actuation and run one step of simulation
        torque_simu = np.zeros(self.rmodel.nv)
        torque_simu[6:] = tau_cmd
        self.simulator.execute(torque_simu)