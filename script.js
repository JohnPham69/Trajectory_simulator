/* =========================================================
   FLIGHT TRAJECTORY SIMULATOR
   Application and Simulation Logic
   ========================================================= */

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

const PROJECTILE_PRESETS = {
    'm982': {
        name: 'M982 Excalibur Shell',
        mass: 48,
        dragCoefficient: 0.3,
        referenceArea: 0.018,
    },
    'cannonball': {
        name: '18th Century Cannonball',
        mass: 5.4, // 12-pounder
        dragCoefficient: 0.47, // Sphere
        referenceArea: 0.0095, // 11cm diameter
    }
};

const ROCKET_DISPLAY_SCALE = 0.25;

/**
 * Simulates the trajectory of a simple shell with constant mass.
 * @param {object} params - The parameters for the simulation.
 * @param {number} params.initialVelocity - The initial velocity of the shell (m/s).
 * @param {number} params.launchAngle - The launch angle in degrees.
 * @param {number} params.launchDirection - The horizontal firing direction in degrees.
 * @param {number} params.dragCoefficient - The drag coefficient.
 * @param {number} params.airDensity - The air density (kg/m^3).
 * @param {number} params.referenceArea - The reference area (m^2).
 * @param {number} params.gravity - Gravitational acceleration (m/s^2).
 * @param {number} params.mass - The mass of the shell (kg).
 * @param {number} params.timeStep - The time step for the simulation (s).
 * @param {number} params.initialElevation - Starting elevation above ground (m).
 * @returns {object} An object containing the full simulation data.
 */
function simulateProjectileTrajectory({ initialVelocity, launchAngle, launchDirection = 0, dragCoefficient, airDensity, referenceArea, mass, gravity, timeStep, initialElevation, groundHeightAtX = () => 0, windVx = 0, windVy = 0, windVz = 0 }) {
    const g = gravity;

    // Convert launch angle to radians
    const launchAngleRad = (launchAngle * Math.PI) / 180;
    const launchDirectionRad = (launchDirection * Math.PI) / 180;
    const horizontalVelocity = initialVelocity * Math.cos(launchAngleRad);

    // Initial conditions
    let position = { x: 0.0, y: initialElevation, z: 0.0 };
    let velocity = {
        vx: horizontalVelocity * Math.cos(launchDirectionRad),
        vy: initialVelocity * Math.sin(launchAngleRad),
        vz: horizontalVelocity * Math.sin(launchDirectionRad)
    };

    const data = [{
        time: 0, ...position, ...velocity, speed: initialVelocity
    }];

    let time = 0;

    while (true) {
        const relativeVx = velocity.vx - windVx;
        const relativeVy = velocity.vy - windVy;
        const relativeVz = velocity.vz - windVz;
        const relativeSpeed = Math.sqrt(relativeVx ** 2 + relativeVy ** 2 + relativeVz ** 2);
        
        // Correctly calculate drag force based on total speed
        const dragForce = 0.5 * dragCoefficient * airDensity * referenceArea * relativeSpeed ** 2;
        
        // Drag acceleration opposes the velocity vector
        const dragAccel = dragForce / mass;
        const ax = relativeSpeed > 1e-6 ? -dragAccel * (relativeVx / relativeSpeed) : 0;
        const ay = relativeSpeed > 1e-6 ? -g - (dragAccel * (relativeVy / relativeSpeed)) : -g;
        const az = relativeSpeed > 1e-6 ? -dragAccel * (relativeVz / relativeSpeed) : 0;

        // Update velocities using Euler integration
        velocity.vx += ax * timeStep;
        velocity.vy += ay * timeStep;
        velocity.vz += az * timeStep;

        // Update positions
        position = {
            x: position.x + velocity.vx * timeStep,
            y: position.y + velocity.vy * timeStep,
            z: position.z + velocity.vz * timeStep
        };

        time += timeStep;

        data.push({
            time,
            ...position,
            ...velocity,
            speed: Math.sqrt(velocity.vx ** 2 + velocity.vy ** 2 + velocity.vz ** 2)
        });

        if (time > timeStep && velocity.vy <= 0 && position.y < groundHeightAtX(position.x, position.z)) {
            break;
        }
    }

    return { data };
}


/**
 * V-2 ROCKET SIMULATION
 * This section contains all functions related to the V-2 rocket trajectory simulation.
 */

const V2_CONSTANTS = {
    g: 9.81, // m/s^2 at sea level
    earthRadius: 6371000, // meters
    R: 287.05, // Specific gas constant for dry air
    gamma: 1.4, // Heat capacity ratio for air
};

const V2_DRAG_COEFFICIENTS = {
    0.0: 0.15, 0.3: 0.16, 0.6: 0.18, 0.8: 0.24, 0.9: 0.33,
    1.0: 0.46, 1.1: 0.42, 1.2: 0.34, 1.5: 0.24, 2.0: 0.18,
    3.0: 0.15, 4.0: 0.14,
};
const machTable = Object.keys(V2_DRAG_COEFFICIENTS).map(Number);
const cdTable = Object.values(V2_DRAG_COEFFICIENTS);

/**
 * Linearly interpolates a value.
 * @param {number} x - The point to interpolate.
 * @param {number[]} xp - The x-coordinates of the data points.
 * @param {number[]} fp - The y-coordinates of the data points.
 * @returns {number} The interpolated value.
 */
function interp(x, xp, fp) {
    const i = xp.findIndex(val => val > x) - 1;
    if (i < 0) return fp[0];
    if (i >= xp.length - 1) return fp[fp.length - 1];
    return fp[i] + (fp[i + 1] - fp[i]) * ((x - xp[i]) / (xp[i + 1] - xp[i]));
}

/**
 * Calculates atmospheric density at a given altitude based on the International Standard Atmosphere model.
 * @param {number} h - Altitude in meters.
 * @returns {number} Air density in kg/m^3.
 */
function atmosphericDensity(h) {
    if (h < 11000) { // Troposphere
        const T = 288.15 - 0.0065 * h;
        const P = 101325 * (T / 288.15) ** 5.255876;
        return P / (V2_CONSTANTS.R * T);
    } else if (h < 20000) { // Lower Stratosphere
        const T = 216.65;
        const P = 22632 * Math.exp(-9.80665 * (h - 11000) / (V2_CONSTANTS.R * T));
        return P / (V2_CONSTANTS.R * T);
    } else { // Upper Stratosphere
        const rho20 = 0.08803;
        const H = 6500.0;
        return rho20 * Math.exp(-(h - 20000) / H);
    }
}

/**
 * Calculates the speed of sound at a given altitude.
 * @param {number} altitude - Altitude in meters.
 * @returns {number} Speed of sound in m/s.
 */
function calculateSpeedOfSound(altitude) {
    let T;
    if (altitude < 11000) {
        T = 288.15 - 0.0065 * altitude;
    } else { // altitude < 20000 or higher
        T = 216.65;
    }
    return Math.sqrt(V2_CONSTANTS.gamma * V2_CONSTANTS.R * T);
}

/**
 * Calculates the Mach number for a given velocity and altitude.
 * @param {number} velocity - The velocity of the object in m/s.
 * @param {number} altitude - The altitude in meters.
 * @returns {number} The Mach number.
 */
function calculateMachNumber(velocity, altitude) {
    const speedOfSound = calculateSpeedOfSound(altitude);
    return speedOfSound > 0 ? Math.abs(velocity) / speedOfSound : 0;
}

/**
 * Calculates the drag coefficient based on Mach number.
 * @param {number} velocity - The velocity of the object in m/s.
 * @param {number} altitude - The altitude in meters.
 * @returns {number} The drag coefficient.
 */
function calculateDragCoefficient(velocity, altitude) {
    let mach = calculateMachNumber(velocity, altitude);
    mach = Math.max(machTable[0], Math.min(mach, machTable[machTable.length - 1]));
    return interp(mach, machTable, cdTable);
}

/**
 * Calculates the total drag force on the rocket.
 * @param {number} velocity - The speed of the rocket in m/s.
 * @param {number} altitude - The altitude in meters.
 * @param {number} frontalArea - The frontal area of the rocket in m^2.
 * @returns {number} The drag force in Newtons.
 */
function calculateDragForce(velocity, altitude, frontalArea) {
    const density = atmosphericDensity(altitude);
    const cd = calculateDragCoefficient(velocity, altitude);
    return 0.5 * density * velocity ** 2 * cd * frontalArea;
}

/**
 * Calculates gravitational acceleration, adjusting for altitude.
 * @param {number} altitude - Altitude in meters.
 * @returns {number} Gravitational acceleration in m/s^2.
 */
function calculateGravity(altitude) {
    return V2_CONSTANTS.g * (V2_CONSTANTS.earthRadius / (V2_CONSTANTS.earthRadius + altitude)) ** 2;
}

/**
 * Determines the rocket's pitch angle (in degrees) based on time.
 * @param {number} time - Time since launch in seconds.
 * @returns {number} Pitch angle in degrees from the horizontal.
 */
function getPitchAngle(time, pitchStart, pitchEnd, startAngle, endAngle) {
    if (time < pitchStart) {
        return startAngle; // Initial pitch
    } else if (time < pitchEnd) {
        // Gravity turn program
        return interp(time, [pitchStart, pitchEnd], [startAngle, endAngle]);
    } else {
        return endAngle; // Final pitch
    }
}

/**
 * Simulates the trajectory of the V-2 rocket.
 * @param {object} params - The parameters for the simulation.
 * @param {number} params.startAngle - Initial pitch angle (degrees).
 * @param {number} params.endAngle - Final pitch angle (degrees).
 * @param {number} params.pitchStart - Time to start pitch program (s).
 * @param {number} params.pitchEnd - Time to end pitch program (s).
 * @param {number} params.timeStep - The time step for the simulation (s).
 * @param {number} params.maxTime - Maximum simulation time (s).
 * @param {number} params.initialMass - Total mass at launch (kg).
 * @param {number} params.endMass - Mass after fuel is spent (kg).
 * @param {number} params.burnTime - Duration of engine burn (s).
 * @param {number} params.frontalArea - Rocket's frontal area (m^2).
 * @param {number} params.thrust - Engine thrust in Newtons (N).
 * @param {number} params.initialElevation - Starting elevation above ground (m).
 * @param {number} params.rocketLaunchDirection - Horizontal firing direction in degrees (default 0).
 * @param {number} [dt=0.1] - The time step for the simulation (s).
 * @param {function} [onProgress] - Optional callback for progress updates.
 * @returns {object} An object containing arrays for times, positions, velocities, and masses.
 */ 
function simulateV2Trajectory({ initialMass, endMass, burnTime, frontalArea, thrust, powerPercent = 100, startAngle, endAngle, pitchStart, pitchEnd, timeStep, maxTime, initialElevation, groundHeightAtX = () => 0, stopAtGround = true, windVx = 0, windVy = 0, windVz = 0, rocketLaunchDirection = 0 }) {
    const dt = timeStep;
    // Initial Conditions
    let position = { x: 0.0, y: initialElevation, z: 0.0 };
    let velocity = { vx: 0.0, vy: 0.0, vz: 0.0 };
    let mass = initialMass;
    const massFlowRate = (initialMass - endMass) / burnTime;
    let time = 0.0;

    // Data storage
    const data = [{
        time,
        ...position,
        ...velocity,
        speed: 0,
        mass,
        mach: 0,
        cd: V2_DRAG_COEFFICIENTS[0],
        pitchAngle: startAngle,
        flightAngle: startAngle
    }];

    let burnoutAltitude = -1;

    // Main simulation loop
    while (true) {
        // Determine current state
        const inBurnPhase = time < burnTime;
        const currentThrust = inBurnPhase ? thrust * (powerPercent / 100) : 0.0;
        if (inBurnPhase) {
            mass = Math.max(endMass, initialMass - massFlowRate * time);
        }
        if (burnoutAltitude < 0 && time >= burnTime) {
            burnoutAltitude = position.y;
        }

        // --- Forces Calculation ---

        // Thrust Vector (with both pitch and azimuth angles)
        const pitchAngle = getPitchAngle(time, pitchStart, pitchEnd, startAngle, endAngle);
        const pitchAngleRad = (pitchAngle * Math.PI) / 180;
        const azimuthRad = (rocketLaunchDirection * Math.PI) / 180;
        const horizontalThrust = currentThrust * Math.cos(pitchAngleRad);
        const thrustVector = {
            x: horizontalThrust * Math.cos(azimuthRad),
            y: currentThrust * Math.sin(pitchAngleRad),
            z: horizontalThrust * Math.sin(azimuthRad)
        };

        // Drag Vector
        const relativeVx = velocity.vx - windVx;
        const relativeVy = velocity.vy - windVy;
        const relativeVz = velocity.vz - windVz;
        const relativeSpeed = Math.sqrt(relativeVx ** 2 + relativeVy ** 2 + relativeVz ** 2);
        let dragVector = { x: 0, y: 0, z: 0 };
        let dragForce = 0;
        if (relativeSpeed > 1e-6) {
            dragForce = calculateDragForce(relativeSpeed, position.y, frontalArea);
            dragVector = {
                x: -dragForce * (relativeVx / relativeSpeed),
                y: -dragForce * (relativeVy / relativeSpeed),
                z: -dragForce * (relativeVz / relativeSpeed)
            };
        }

        // Gravity Vector
        const gravityVector = {
            x: 0.0,
            y: -mass * calculateGravity(position.y)
        };

        // Total Force and Acceleration
        const totalForce = {
            x: thrustVector.x + dragVector.x + gravityVector.x,
            y: thrustVector.y + dragVector.y + gravityVector.y,
            z: thrustVector.z + dragVector.z
        };
        const acceleration = {
            x: totalForce.x / mass,
            y: totalForce.y / mass,
            z: totalForce.z / mass
        };

        // --- Euler Integration ---
        velocity.vx += acceleration.x * dt;
        velocity.vy += acceleration.y * dt;
        velocity.vz += acceleration.z * dt;
        position.x += velocity.vx * dt;
        position.y += velocity.vy * dt;
        position.z += velocity.vz * dt;

        time += dt;

        // --- Data Storage & Progress ---
        const speed = Math.sqrt(velocity.vx ** 2 + velocity.vy ** 2 + velocity.vz ** 2);
        const mach = calculateMachNumber(relativeSpeed, position.y);
        const cd = calculateDragCoefficient(relativeSpeed, position.y);
        const flightAngle = speed > 1e-6
            ? THREE.MathUtils.radToDeg(Math.atan2(velocity.vy, velocity.vx))
            : pitchAngle;
        data.push({
            time,
            ...position,
            ...velocity,
            speed,
            mass,
            mach,
            cd,
            pitchAngle,
            flightAngle
        });

        if (stopAtGround && time > dt && velocity.vy <= 0 && position.y < groundHeightAtX(position.x, position.z)) {
            break;
        }
        
        // Safety break for very long or failed simulations
        if (time > maxTime) { 
            console.warn(`Simulation exceeded max time of ${maxTime}s, terminating.`);
            break;
        }
    }

    return { data, burnoutAltitude };
}


/* =========================================================
   UI AND APPLICATION LOGIC
   ========================================================= */

document.addEventListener('DOMContentLoaded', () => {
    // --- STATE ---
    let state = {
        simulationType: 'projectile', // 'projectile' or 'rocket'
        simulationData: null,
        target: null, // {x, y} for the game target
        animationFrameId: null,
        previewFrameId: null,
        pendingSimulationParams: null,
        activeExplosions: [], // Holds our particle systems once spawned
        charts: {},
        isDarkMode: window.matchMedia('(prefers-color-scheme: dark)').matches
    };

    // --- DOM & 3D ELEMENTS ---
    let three = { scene: null, camera: null, renderer: null, controls: null, projectile: null, ground: null, line: null, scenery: null, windIndicator: null, targetMarker: null, mapPointerDown: null, mapPointerMoved: false, launchElevation: 0, launchOriginX: 0, worldScale: 1 };
    const impactSound = new Audio('./sound_effect/mixkit-war-explosions-2773.wav');

    function playImpactSound() {
        impactSound.currentTime = 0;
        impactSound.play().catch(() => {});
    }

    const DOMElements = {
        themeToggle: document.getElementById('theme-toggle'),
        projectileTab: document.getElementById('projectile-tab'),
        rocketTab: document.getElementById('rocket-tab'),
        projectilePanel: document.getElementById('projectile-panel'),
        rocketPanel: document.getElementById('rocket-panel'),
        projectileType: document.getElementById('projectile-type'),
        view3DToggle: null, // Will be created dynamically
        projectileForm: document.getElementById('projectile-form'),
        rocketForm: document.getElementById('rocket-form'),
        runButton: document.getElementById('run-simulation'),
        mobileControlsToggle: document.getElementById('mobile-controls-toggle'),
        mobileControlsClose: document.getElementById('mobile-controls-close'),
        mobileControlsBackdrop: document.getElementById('mobile-controls-backdrop'),
        controlPanel: document.querySelector('.control-panel'),
        resetButton: document.getElementById('reset-simulation'),
        exportButton: document.getElementById('export-csv'),
        formError: document.getElementById('form-error'),
        statusBadge: document.getElementById('simulation-status'),
        rocketOnlyElements: document.querySelectorAll('.rocket-only'),
        summary: {
            maxAltitude: document.getElementById('maximum-altitude'),
            // Rocket live readouts
            rocketFlightPhase: document.getElementById('rocket-flight-phase'),
            rocketLiveReadout: document.getElementById('rocket-live-readout'),
            rocketPowerOutput: document.getElementById('rocket-power-output'),
            rocketPowerSlider: document.getElementById('rocket-power'),
            // Summary cards
            range: document.getElementById('horizontal-range'),
            flightTime: document.getElementById('flight-time'),
            maxSpeed: document.getElementById('maximum-speed'),
            maxMach: document.getElementById('maximum-mach'),
        },
        table: {
            head: document.getElementById('simulation-table-head'),
            body: document.getElementById('simulation-table-body'),
            limit: document.getElementById('table-row-limit'),
        },
        resetGraphButton: document.getElementById('reset-graph-button'),
        chartResetButtons: document.querySelectorAll('.chart-reset-button'),
        profileButtons: document.querySelectorAll('.profile-button'),
        animationCanvas: document.getElementById('animation-canvas'),
        get animationCtx() {
            return this.animationCanvas.getContext('2d');
        },
        threeCanvas: document.getElementById('three-canvas'),
        trajectoryChartCanvas: document.getElementById('trajectory-chart'),
    };

    // --- CHART CONFIGURATION ---
    const chartConfigs = {
        trajectory: {
            type: 'scatter',
            options: {
                scales: {
                    x: {
                        title: { display: true, text: 'Range (m)' },
                        min: 0,
                        max: 20000 // Set a larger default max
                    },
                    y: {
                        title: { display: true, text: 'Altitude (m)' },
                        min: 0,
                        max: 20000 // Set a larger default max
                    }
                }
            }
        },
        velocity: {
            type: 'line',
            options: { scales: { x: { title: { display: true, text: 'Time (s)' } }, y: { title: { display: true, text: 'Velocity (m/s)' } } } }
        },
        altitude: {
            type: 'line',
            options: { scales: { x: { title: { display: true, text: 'Time (s)' } }, y: { title: { display: true, text: 'Altitude (m)' } } } }
        },
        mass: {
            type: 'line',
            options: { scales: { x: { title: { display: true, text: 'Time (s)' } }, y: { title: { display: true, text: 'Mass (kg)' } } } }
        },
        mach: {
            type: 'line',
            options: { scales: { x: { title: { display: true, text: 'Time (s)' } }, y: { title: { display: true, text: 'Value' } } } }
        }
    };

    const chartColors = {
        blue: 'rgba(33, 85, 205, 0.8)',
        sky: 'rgba(100, 150, 255, 0.8)',
        green: 'rgba(22, 121, 79, 0.8)',
        orange: 'rgba(255, 159, 64, 0.8)',
        red: 'rgba(255, 99, 132, 0.8)',
    };

    // --- INITIALIZATION ---
    function init() {
        setupEventListeners();
        applyTheme(state.isDarkMode);
        populateProjectilePresets();
        create3DViewToggle();
        if (window.matchMedia('(max-width: 680px)').matches) closeMobileControls();
        resetUI();
        updateCharts([]); // Create empty charts on load so target can be set
    }

    function create3DViewToggle() {
        const container = document.createElement('div');
        container.className = 'view-toggle';

        const checkbox = document.createElement('input');
        checkbox.type = 'checkbox';
        checkbox.id = 'view-3d-toggle';
        DOMElements.view3DToggle = checkbox;

        const label = document.createElement('label');
        label.htmlFor = 'view-3d-toggle';
        label.textContent = 'Enable 3D View';
        label.style.fontWeight = 'bold';

        container.append(checkbox, label);
        DOMElements.projectilePanel.parentElement.insertBefore(container, DOMElements.projectilePanel);
    }
    // --- EVENT LISTENERS ---
    function setupEventListeners() {
        DOMElements.trajectoryChartCanvas.addEventListener('click', onChartClick);
        DOMElements.threeCanvas.addEventListener('pointerdown', on3DMapPointerDown);
        DOMElements.threeCanvas.addEventListener('pointermove', on3DMapPointerMove);
        DOMElements.threeCanvas.addEventListener('pointerup', on3DMapPointerUp);
        DOMElements.threeCanvas.addEventListener('pointercancel', on3DMapPointerCancel);
        DOMElements.threeCanvas.addEventListener('click', on3DMapClick);
        DOMElements.themeToggle.addEventListener('click', toggleTheme);
        DOMElements.projectileTab.addEventListener('click', () => switchSimulationType('projectile'));
        DOMElements.rocketTab.addEventListener('click', () => switchSimulationType('rocket'));
        DOMElements.runButton.addEventListener('click', runSimulation);
        DOMElements.mobileControlsToggle.addEventListener('click', toggleMobileControls);
        DOMElements.mobileControlsClose.addEventListener('click', closeMobileControls);
        DOMElements.mobileControlsBackdrop.addEventListener('click', closeMobileControls);
        DOMElements.resetButton.addEventListener('click', resetUI);
        DOMElements.exportButton.addEventListener('click', exportToCSV);
        DOMElements.table.limit.addEventListener('change', () => updateTable(state.simulationData));
        DOMElements.resetGraphButton.addEventListener('click', resetGraph);
        DOMElements.projectileType.addEventListener('change', onProjectileTypeChange);
        DOMElements.chartResetButtons.forEach(button => {
            button.addEventListener('click', () => {
                const chartId = button.dataset.chart;
                if (state.charts[chartId]) {
                    state.charts[chartId].resetZoom();
                }
            });
        });
        DOMElements.profileButtons.forEach(button => {
            button.addEventListener('click', onProfileButtonClick);
        });
        DOMElements.summary.rocketPowerSlider.addEventListener('input', onPowerSliderChange);
        document.addEventListener('keydown', onKeyDown);
    }

    function toggleMobileControls() {
        const isOpen = DOMElements.controlPanel.classList.toggle('mobile-controls-open');
        DOMElements.mobileControlsToggle.setAttribute('aria-expanded', isOpen);
        DOMElements.controlPanel.setAttribute('aria-hidden', !isOpen);
    }

    function closeMobileControls() {
        DOMElements.controlPanel.classList.remove('mobile-controls-open');
        DOMElements.mobileControlsToggle.setAttribute('aria-expanded', 'false');
        DOMElements.controlPanel.setAttribute('aria-hidden', 'true');
    }

    function onChartClick(event) {
        const chart = state.charts.trajectory;
        if (!chart || state.animationFrameId) return; // Don't set target while running
    
        const rect = chart.canvas.getBoundingClientRect();
        const x = event.clientX - rect.left;
        const y = event.clientY - rect.top;
    
        // Convert pixel coordinates to data coordinates
        const dataX = chart.scales.x.getValueForPixel(x);
        const dataY = chart.scales.y.getValueForPixel(y);
    
        // Ensure target is within the valid gameplay area
        if (dataX < 0 || dataY < 0) return;
    
        state.target = { x: dataX, y: dataY };
        drawTarget(); // Draw the new target immediately
        if (state.simulationType === 'projectile') {
            setUIState('target-selected');
        }
    }

    function on3DMapClick(event) {
        if (!three.renderer || !three.camera || !three.ground || state.animationFrameId || three.mapPointerMoved) return;

        const rect = DOMElements.threeCanvas.getBoundingClientRect();
        const pointer = new THREE.Vector2(
            ((event.clientX - rect.left) / rect.width) * 2 - 1,
            -((event.clientY - rect.top) / rect.height) * 2 + 1
        );
        const raycaster = new THREE.Raycaster();
        raycaster.setFromCamera(pointer, three.camera);

        const intersections = raycaster.intersectObject(three.ground, false);
        if (intersections.length === 0) return;

        const hit = intersections[0].point;
        state.target = {
            x: (hit.x - three.launchOriginX) / three.worldScale,
            y: (hit.y - three.launchElevation) / three.worldScale,
            worldX: hit.x,
            worldY: hit.y,
            worldZ: hit.z
        };
        show3DTargetMarker(hit);
        setUIState('target-selected');
    }

    function on3DMapPointerDown(event) {
        three.mapPointerDown = { x: event.clientX, y: event.clientY };
        three.mapPointerMoved = false;
    }

    function on3DMapPointerMove(event) {
        if (!three.mapPointerDown || three.mapPointerMoved) return;

        const distance = Math.hypot(
            event.clientX - three.mapPointerDown.x,
            event.clientY - three.mapPointerDown.y
        );
        if (distance > 8) three.mapPointerMoved = true;
    }

    function on3DMapPointerUp() {
        three.mapPointerDown = null;
    }

    function on3DMapPointerCancel() {
        three.mapPointerDown = null;
        three.mapPointerMoved = true;
    }

    function onProfileButtonClick(event) {
        const button = event.currentTarget;
        const profile = button.dataset.profile;

        DOMElements.profileButtons.forEach(btn => btn.classList.remove('active'));
        button.classList.add('active');

        document.getElementById('rocket-profile').value = profile;
        // You might want to update some rocket parameters here based on the profile
    }

    function onPowerSliderChange(event) {
        DOMElements.summary.rocketPowerOutput.textContent = `${event.currentTarget.value}%`;
    }

    /**
     * Automatically generates random wind speed and direction values.
     * Wind speed: 1-10 m/s
     * Wind direction: 0-50°
     */
    function autoRandomizeWind() {
        const windSpeedField = document.getElementById('projectile-wind-speed');
        const windDirectionField = document.getElementById('projectile-wind-direction');
        
        // Generate random wind speed (1-10 m/s)
        const randomSpeed = Math.random() * 9 + 1;
        windSpeedField.value = randomSpeed.toFixed(1);
        
        // Generate random wind direction (0-50°)
        const randomDirection = Math.random() * 50;
        windDirectionField.value = randomDirection.toFixed(0);
    }

    function onKeyDown(event) {
        // OrbitControls handles camera movement, so this is no longer needed.
    }

    function resetGraph() {
        state.target = null;
        state.simulationData = null;
        resetUI(false); // Soft reset
        updateCharts([]); // Redraw empty charts
    }

    function populateProjectilePresets() {
        const select = DOMElements.projectileType;
        select.innerHTML = '';
        for (const key in PROJECTILE_PRESETS) {
            const option = document.createElement('option');
            option.value = key;
            option.textContent = PROJECTILE_PRESETS[key].name;
            select.appendChild(option);
        }
        onProjectileTypeChange(); // Set initial values
    }

    function onProjectileTypeChange() {
        updateProjectileInputs();
    }

    // --- THEME ---
    function toggleTheme() {
        state.isDarkMode = !state.isDarkMode;
        applyTheme(state.isDarkMode);
    }

    function applyTheme(isDark) {
        document.body.classList.toggle('dark-theme', isDark);
        DOMElements.themeToggle.setAttribute('aria-pressed', isDark);
        DOMElements.themeToggle.textContent = isDark ? 'Light mode' : 'Dark mode';
        // Re-render charts with correct colors if they exist
        Object.values(state.charts).forEach(chart => chart.destroy());
        state.charts = {};
        if (state.simulationData) {
            updateCharts(state.simulationData, true);
        }
    }

    // --- SIMULATION TYPE ---
    function switchSimulationType(type) {
        state.simulationType = type;
        const isProjectile = type === 'projectile';

        DOMElements.projectileTab.classList.toggle('active', isProjectile);
        DOMElements.projectileTab.setAttribute('aria-selected', isProjectile);
        DOMElements.rocketTab.classList.toggle('active', !isProjectile);
        DOMElements.rocketTab.setAttribute('aria-selected', !isProjectile);

        DOMElements.projectilePanel.hidden = !isProjectile;
        DOMElements.rocketPanel.hidden = isProjectile;

        DOMElements.rocketOnlyElements.forEach(el => el.hidden = isProjectile);

        resetUI();
    }

    // --- SIMULATION EXECUTION ---
    function runSimulation() {
        // Auto-randomize wind on each simulation run
        autoRandomizeWind();
        
        if (state.animationFrameId) { // If animation is running, stop it
            cancelAnimationFrame(state.animationFrameId);
            state.animationFrameId = null;
            setUIState('ready');
            return;
        }

        const is3D = DOMElements.view3DToggle.checked;

        const isPrepared3D = is3D && state.pendingSimulationParams;
        let params = state.pendingSimulationParams;

        if (!params || (isPrepared3D && state.simulationType === 'projectile' && state.target)) {
            const form = state.simulationType === 'projectile' ? DOMElements.projectileForm : DOMElements.rocketForm;
            const formParams = getAndValidateFormParams(form);

            if (!formParams) return;

            params = isPrepared3D
                ? {
                    ...formParams,
                    initialElevation: 0,
                    worldLaunchElevation: state.pendingSimulationParams.worldLaunchElevation,
                    worldOriginX: state.pendingSimulationParams.worldOriginX,
                    worldScale: state.pendingSimulationParams.worldScale
                }
                : formParams;

            // Keep the physics origin at ground level. 3D applies the map elevation visually.
            params.initialElevation = 0;
            if (!isPrepared3D) {
                params.worldLaunchElevation = 0;
                params.worldOriginX = state.simulationType === 'rocket' ? -26000 : 0;
            }
        }

        const windDirectionRad = (params.windDirection * Math.PI) / 180;
        params.windVx = params.windSpeed * Math.cos(windDirectionRad);
        params.windVz = params.windSpeed * Math.sin(windDirectionRad);

        if (is3D && !isPrepared3D) {
            resetUI(false);
            state.pendingSimulationParams = params;
            DOMElements.trajectoryChartCanvas.style.display = 'none';
            DOMElements.threeCanvas.style.display = 'block';
            params.worldScale = state.simulationType === 'rocket' ? ROCKET_DISPLAY_SCALE : 1;
            three.worldScale = params.worldScale;
            three.launchOriginX = params.worldOriginX;
            init3DScene(params);
            params.worldLaunchElevation = three.launchElevation;
            three.projectile.position.set(three.launchOriginX, params.worldLaunchElevation, 0);
            start3DPreview();
            setUIState('map-ready');
            return;
        }

        state.pendingSimulationParams = null;
        if (state.previewFrameId) {
            cancelAnimationFrame(state.previewFrameId);
            state.previewFrameId = null;
        }

        setUIState('running');
        if (!isPrepared3D) {
            resetUI(false);
        } else {
            state.simulationData = null;
            state.activeExplosions = [];
        }

        // Initialize charts before starting the animation
        updateCharts([]);

        // Use setTimeout to allow the UI to update before the heavy computation
        setTimeout(() => {
            try {
                if (is3D) {
                    DOMElements.trajectoryChartCanvas.style.display = 'none';
                    DOMElements.threeCanvas.style.display = 'block';
                    if (!three.renderer) init3DScene(params);
                    three.renderer.render(three.scene, three.camera);
                } else {
                    DOMElements.trajectoryChartCanvas.style.display = 'block';
                    DOMElements.threeCanvas.style.display = 'none';
                }

                params.groundHeightAtX = is3D
                    ? (x, z = 0) => getTerrainHeight(three.launchOriginX + x * three.worldScale, z * three.worldScale) - params.worldLaunchElevation
                    : () => 0;
                params.stopAtGround = !(is3D && state.simulationType === 'rocket');

                if (is3D) {
                    three.projectile.position.set(three.launchOriginX, params.worldLaunchElevation, 0);
                }

                // Run the appropriate simulation to get the static data array first.
                const simulationResult = state.simulationType === 'projectile'
                    ? simulateProjectileTrajectory(params)
                    : simulateV2Trajectory(params);

                // Start the frame-by-frame playback animation.
                playbackAnimation(simulationResult, params.worldLaunchElevation);

            } catch (error) {
                console.error("Simulation failed:", error);
                setUIState('error', 'Simulation failed. Check console for details.');
            }
        }, 50);
    }

    /**
     * Animates a pre-calculated trajectory frame by frame.
     * @param {object} simulation - The result object from a simulation function.
     * @param {Array<object>} simulation.data - The array of trajectory data points.
     */
    function playbackAnimation(simulation, worldLaunchElevation = 0) {
        const trajectoryData = simulation.data;
        if (!trajectoryData || trajectoryData.length === 0) {
            setUIState('error', 'Simulation produced no data.');
            return;
        }

        const is3D = DOMElements.view3DToggle.checked;
        const isProjectile = state.simulationType === 'projectile';
        state.simulationData = trajectoryData; // Store full data for other UI components

        // --- Playback state tracking ---
        let currentFrameIndex = 0;

        const trajectoryPoints = [];
        let terrainImpactPoint = null;
        const terrainCollisionSafeTime = 0.15;
        const trajectoryPointStride = is3D
            ? Math.max(1, Math.ceil(trajectoryData.length / 4000))
            : 1;

        // --- Chart references ---
        const trajectoryChart = state.charts.trajectory;
        const velocityChart = state.charts.velocity;
        const altitudeChart = state.charts.altitude;
        const massChart = isProjectile ? null : state.charts.mass;
        const machChart = isProjectile ? null : state.charts.mach;

        // --- Live UI update helper ---
        function updateLiveUI(dataPoint, currentFrameIndex) {
            // Calculate running max values based on data seen so far
            const visibleData = trajectoryData.slice(0, currentFrameIndex + 1);
            const currentMaxAltitude = Math.max(...visibleData.map(p => p.y));
            const currentMaxSpeed = Math.max(...visibleData.map(p => p.speed));
            const currentMaxMach = isProjectile ? 0 : Math.max(...visibleData.map(p => p.mach || 0));
            
            updateLiveSummary(dataPoint, currentMaxAltitude, currentMaxSpeed, currentMaxMach);
            if (!isProjectile) {
                // Use a reasonable default for burnTime if not in simulation data
                const burnTime = simulation.data[simulation.data.length - 1]?.burnTime || 65;
                const phase = dataPoint.time < burnTime ? "Boost Phase" : "Coast Phase";
                const readout = `Alt: ${dataPoint.y.toFixed(0)}m, Vel: ${dataPoint.speed.toFixed(1)}m/s`;
                DOMElements.summary.rocketFlightPhase.textContent = phase;
                DOMElements.summary.rocketLiveReadout.textContent = readout;
            }
        }
        
        // Pre-calculate max values for reference (not used in live updates anymore)
        const maxValues = {
            maxAltitude: Math.max(...trajectoryData.map(p => p.y)),
            maxSpeed: Math.max(...trajectoryData.map(p => p.speed)),
            maxMach: isProjectile ? 0 : Math.max(...trajectoryData.map(p => p.mach || 0)),
        };

        function updateAnimatedLineCharts(frameCount) {
            const visibleData = trajectoryData.slice(0, frameCount);
            const labels = visibleData.map(point => point.time.toFixed(1));

            velocityChart.data.labels = labels;
            velocityChart.data.datasets[0].data = visibleData.map(point => point.speed);
            velocityChart.data.datasets[1].data = visibleData.map(point => point.vx);
            velocityChart.data.datasets[2].data = visibleData.map(point => point.vy);
            velocityChart.update('none');

            altitudeChart.data.labels = labels;
            altitudeChart.data.datasets[0].data = visibleData.map(point => point.y);
            altitudeChart.update('none');

            if (!isProjectile) {
                massChart.data.labels = labels;
                massChart.data.datasets[0].data = visibleData.map(point => point.mass);
                massChart.update('none');

                machChart.data.labels = labels;
                machChart.data.datasets[0].data = visibleData.map(point => point.mach);
                machChart.data.datasets[1].data = visibleData.map(point => point.cd);
                machChart.update('none');
            }
        }

        function animationLoop() {
            state.animationFrameId = requestAnimationFrame(animationLoop);

            // Always update the 3D scene if it's active
            if (is3D) three.renderer.render(three.scene, three.camera);

            // SCENARIO A: Projectile is flying
            if (currentFrameIndex < trajectoryData.length) {
                const frameData = trajectoryData[currentFrameIndex];

                // Update UI and visuals
                updateLiveUI(frameData, currentFrameIndex);
                if (is3D) {
                    const isLastFrame = currentFrameIndex === trajectoryData.length - 1;
                    let visualFrameData = frameData;

                    const nextPosition = new THREE.Vector3(
                        three.launchOriginX + frameData.x * three.worldScale,
                        frameData.y * three.worldScale + worldLaunchElevation,
                        frameData.z * three.worldScale
                    );
                    const previousFrameData = trajectoryData[Math.max(0, currentFrameIndex - 1)];
                    const previousWorldX = three.launchOriginX + previousFrameData.x * three.worldScale;
                    const previousWorldY = previousFrameData.y * three.worldScale + worldLaunchElevation;
                    const previousWorldZ = previousFrameData.z * three.worldScale;
                    const previousGroundHeight = getTerrainHeight(previousWorldX, previousWorldZ);
                    const nextGroundHeight = getTerrainHeight(nextPosition.x, nextPosition.z);
                    const previousClearance = previousWorldY - previousGroundHeight;
                    const nextClearance = nextPosition.y - nextGroundHeight;
                    const crossesTerrain = currentFrameIndex > 0
                        && previousClearance > 0
                        && nextClearance <= 0
                        && frameData.vy <= 0;
                    const collisionPoint = frameData.time > terrainCollisionSafeTime && crossesTerrain
                        ? findTerrainCollision(
                            new THREE.Vector3(previousWorldX, previousWorldY, previousWorldZ),
                            nextPosition
                        ) || new THREE.Vector3(nextPosition.x, nextGroundHeight, nextPosition.z)
                        : null;
                    if (collisionPoint) {
                        terrainImpactPoint = collisionPoint;
                        visualFrameData = {
                            ...frameData,
                            x: (collisionPoint.x - three.launchOriginX) / three.worldScale,
                            y: (collisionPoint.y - worldLaunchElevation) / three.worldScale
                        };
                    }
                    if (currentFrameIndex % trajectoryPointStride === 0 || isLastFrame || collisionPoint) {
                        trajectoryPoints.push(new THREE.Vector3(
                            three.launchOriginX + visualFrameData.x * three.worldScale,
                            visualFrameData.y * three.worldScale + worldLaunchElevation,
                            visualFrameData.z * three.worldScale
                        ));
                    }
                    update3DScene(visualFrameData, trajectoryPoints, worldLaunchElevation, three.worldScale, three.launchOriginX);
                } else {
                    drawAnimatedProjectile(frameData, trajectoryChart);
                    trajectoryChart.data.datasets[0].data = trajectoryData
                        .slice(0, currentFrameIndex + 1)
                        .map(point => ({ x: point.x, y: point.y }));
                    trajectoryChart.update('none');
                }
                updateAnimatedLineCharts(currentFrameIndex + 1);

                currentFrameIndex++;

                if (terrainImpactPoint && is3D) {
                    currentFrameIndex = trajectoryData.length;
                }

                // IMPACT CHECK: Trigger on the exact frame the loop hits the final index
                if (currentFrameIndex === trajectoryData.length) {
                    if (is3D) playImpactSound();
                    setUIState('complete');
                    updateSummary(simulation);
                    updateTable(trajectoryData);

                    if (is3D) {
                        // Hide the shell mesh
                        three.projectile.visible = false;
                        // Spawn a final blast at the impact point
                        const finalPos = terrainImpactPoint || new THREE.Vector3(
                            three.launchOriginX + trajectoryData[trajectoryData.length - 1].x * three.worldScale,
                            getTerrainHeight(
                                three.launchOriginX + trajectoryData[trajectoryData.length - 1].x * three.worldScale,
                                trajectoryData[trajectoryData.length - 1].z * three.worldScale
                            ),
                            trajectoryData[trajectoryData.length - 1].z * three.worldScale
                        );
                        state.activeExplosions.push(createBlast(finalPos.x, finalPos.y, finalPos.z));
                    }
                }
            }

            // SCENARIO B: Projectile has hit; animate the active particle system
            // Animate all active explosions and clean up faded ones
            state.activeExplosions = state.activeExplosions.filter(explosion => {
                if (explosion.material.opacity <= 0) {
                    // Cleanup memory
                    three.scene.remove(explosion.mesh);
                    explosion.geometry.dispose();
                    explosion.material.dispose();
                    return false; // Remove from array
                }

                const positionsAttr = explosion.geometry.attributes.position.array;

                for (let i = 0; i < explosion.count; i++) {
                    // Expand individual particle vectors outward
                    positionsAttr[i * 3] += explosion.velocities[i].x * 0.1;
                    positionsAttr[i * 3 + 1] += explosion.velocities[i].y * 0.1;
                    positionsAttr[i * 3 + 2] += explosion.velocities[i].z * 0.1;
                    // Pull particles downward with simulated gravity
                    explosion.velocities[i].y -= 2.5;
                }
                explosion.geometry.attributes.position.needsUpdate = true;

                // Smoothly fade out opacity
                explosion.material.opacity -= 0.015;
                return true; // Keep in array
            });

            // If playback is finished and there's no explosion, stop the loop.
            // For 3D, we keep the loop running for camera controls.
            if (currentFrameIndex >= trajectoryData.length && state.activeExplosions.length === 0 && !is3D) {
                cancelAnimationFrame(state.animationFrameId);
                state.animationFrameId = null;
                
                // Final UI updates for 2D mode
                updateCharts(trajectoryData, true);
                setUIState('complete');
                updateSummary(simulation);
                updateTable(trajectoryData);
            } else if (is3D) {
                three.controls.update(); // Keep controls interactive
            }
        }

        // Populate charts with full data at the start for a static line
        updateCharts(trajectoryData, true);
        if (!is3D) {
            // For 2D, clear the trajectory line so we can animate it point by point
            state.charts.trajectory.data.datasets[0].data = [];
            state.charts.trajectory.update('none');
        }
        state.charts.velocity.data.labels = [];
        state.charts.velocity.data.datasets.forEach(dataset => dataset.data = []);
        state.charts.velocity.update('none');
        state.charts.altitude.data.labels = [];
        state.charts.altitude.data.datasets[0].data = [];
        state.charts.altitude.update('none');
        if (!isProjectile) {
            state.charts.mass.data.labels = [];
            state.charts.mass.data.datasets[0].data = [];
            state.charts.mass.update('none');
            state.charts.mach.data.labels = [];
            state.charts.mach.data.datasets.forEach(dataset => dataset.data = []);
            state.charts.mach.update('none');
        }

        animationLoop();
    }

    function start3DPreview() {
        const renderPreview = () => {
            if (!three.renderer || !three.controls) return;

            three.controls.update();
            three.renderer.render(three.scene, three.camera);
            state.previewFrameId = requestAnimationFrame(renderPreview);
        };

        renderPreview();
    }

    /**
     * Calculates the terrain height at a given world position.
     * This function must match the terrain generation logic in init3DScene.
     * @param {number} x - The world x-coordinate.
     * @param {number} z - The world z-coordinate.
     * @returns {number} The terrain height (y-coordinate) at that point.
     */
    function getTerrainHeight(x, z) {
        const height = 1500 * (Math.sin(x / 5000) * Math.cos(z / 5000));
        return height - 500; // Matches the ground's y-position offset
    }

    function getAndValidateFormParams(form) {
        const formData = new FormData(form);
        const params = {};
        let isValid = true;
        let errorMessage = '';

        // For projectile, add preset values to params
        if (state.simulationType === 'projectile') {
            const presetKey = DOMElements.projectileType.value;
            if (PROJECTILE_PRESETS[presetKey]) {
                const preset = PROJECTILE_PRESETS[presetKey];
                // These are now read from the advanced inputs
            }
        }

        for (const [name, value] of formData.entries()) {
            const input = form.elements[name];

            // Skip non-numeric validation for these fields
            if (['projectileType', 'rocketPreset', 'rocketProfile', 'randomizeWindSpeed', 'randomizeWindDirection', 'playbackSpeed'].includes(name)) {
                continue;
            }

            const numValue = parseFloat(value);

            if (isNaN(numValue) || value.trim() === '') {
                errorMessage = `Invalid value for ${input.labels[0].textContent}. Must be a number.`;
                isValid = false;
                break;
            }

            const min = parseFloat(input.min);
            const max = parseFloat(input.max);

            if (!isNaN(min) && numValue < min) {
                errorMessage = `${input.labels[0].textContent} must be at least ${min}.`;
                isValid = false;
                break;
            }
            if (!isNaN(max) && numValue > max) {
                errorMessage = `${input.labels[0].textContent} must not exceed ${max}.`;
                isValid = false;
                break;
            }

            params[name] = numValue;
        }

        // Specific cross-field validation
        if (params.endMass && params.initialMass && params.endMass >= params.initialMass) {
            errorMessage = 'Final mass must be less than initial mass.';
            isValid = false;
        }
        if (params.pitchStart && params.pitchEnd && params.pitchStart >= params.pitchEnd) {
            errorMessage = 'Pitch program start time must be before end time.';
            isValid = false;
        }

        DOMElements.formError.textContent = errorMessage;
        DOMElements.formError.hidden = isValid;

        return isValid ? params : null;
    }

    // --- UI STATE & RESET ---
    function setUIState(status, message = '') {
        const isSimulating = status === 'running';
        DOMElements.runButton.disabled = false; // Always enable run/stop
        DOMElements.resetButton.disabled = isSimulating;
        DOMElements.exportButton.disabled = isSimulating || !state.simulationData;

        // Disable forms during simulation
        DOMElements.projectileForm.disabled = isSimulating;
        DOMElements.rocketForm.disabled = isSimulating;

        DOMElements.statusBadge.classList.remove('success', 'error', 'running');
        switch (status) {
            case 'running':
                DOMElements.statusBadge.textContent = 'Running...';
                DOMElements.statusBadge.classList.add('running');
                DOMElements.runButton.textContent = 'Stop Simulation';
                DOMElements.runButton.classList.add('stop-button');
                break;
            case 'armed':
                DOMElements.statusBadge.textContent = 'Map Ready';
                DOMElements.runButton.textContent = 'Shoot!';
                DOMElements.runButton.classList.remove('stop-button');
                break;
            case 'map-ready':
                DOMElements.statusBadge.textContent = 'Select Target';
                DOMElements.runButton.textContent = 'Select target on map';
                DOMElements.runButton.disabled = true;
                DOMElements.runButton.classList.remove('stop-button');
                break;
            case 'target-selected':
                DOMElements.statusBadge.textContent = 'Target Selected';
                DOMElements.runButton.textContent = 'Shoot!';
                DOMElements.runButton.disabled = false;
                DOMElements.runButton.classList.remove('stop-button');
                DOMElements.formError.hidden = true;
                break;
            case 'complete':
                DOMElements.statusBadge.textContent = 'Complete';
                DOMElements.statusBadge.classList.add('success');
                DOMElements.runButton.textContent = 'Run Simulation';
                DOMElements.runButton.classList.remove('stop-button');
                break;
            case 'hit':
                DOMElements.statusBadge.textContent = 'Target Hit!';
                DOMElements.statusBadge.classList.add('success');
                DOMElements.runButton.textContent = 'Run Simulation';
                DOMElements.runButton.classList.remove('stop-button');
                break;
            case 'error':
                if (state.animationFrameId) {
                    cancelAnimationFrame(state.animationFrameId);
                    state.animationFrameId = null;
                }
                DOMElements.runButton.textContent = 'Run Simulation';
                DOMElements.runButton.classList.remove('stop-button');
                DOMElements.statusBadge.textContent = 'Error';
                DOMElements.statusBadge.classList.add('error');
                DOMElements.formError.textContent = message;
                DOMElements.formError.hidden = false;
                break;
            default: // 'ready'
                DOMElements.statusBadge.textContent = 'Ready';
                DOMElements.runButton.textContent = 'Run Simulation';
                DOMElements.runButton.classList.remove('stop-button');
                break;
        }
    }

    function resetUI(hardReset = true) {
        if (state.animationFrameId) {
            cancelAnimationFrame(state.animationFrameId);
            state.animationFrameId = null;
        }
        if (state.previewFrameId) {
            cancelAnimationFrame(state.previewFrameId);
            state.previewFrameId = null;
        }
        state.activeExplosions = [];
        state.simulationData = null;
        state.pendingSimulationParams = null;
        if (hardReset) state.target = null;

        // Reset canvases
        DOMElements.trajectoryChartCanvas.style.display = 'block';
        DOMElements.threeCanvas.style.display = 'none';
        if (three.renderer) {
            three.renderer.dispose();
            three.renderer = null;
            if (three.controls) three.controls.dispose();
            disposeWindIndicator();
            dispose3DTargetMarker();
            DOMElements.threeCanvas.innerHTML = '';
        }

        DOMElements.animationCtx.clearRect(0, 0, DOMElements.animationCanvas.width, DOMElements.animationCanvas.height);
        
        // Reset rocket live readouts
        if (state.simulationType === 'rocket') {
            DOMElements.summary.rocketFlightPhase.textContent = 'Ready for launch';
            DOMElements.summary.rocketLiveReadout.textContent = 'Select a profile and run the simulation.';
        }

        // Reset summary cards
        Object.values(DOMElements.summary).forEach(el => el.textContent = '—');

        // Clear charts
        Object.values(state.charts).forEach(chart => chart.destroy());
        state.charts = {};

        // Clear table
        DOMElements.table.body.innerHTML = `<tr><td colspan="8" class="empty-table-message">Run a simulation to view calculated data.</td></tr>`;
        updateTableHeaders();

        if (hardReset) {
            // DOMElements.projectileForm.reset(); // Can be annoying
            // DOMElements.rocketForm.reset();
            if (state.target) drawTarget(); // Redraw target if it exists
            updateProjectileInputs(); // Restore defaults
        }

        DOMElements.formError.hidden = true;
        setUIState('ready');
    }
    
    function redrawOverlay() {
        const ctx = DOMElements.animationCtx;
        ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
        if (state.target) {
            drawTarget();
        }
    }

    function updateProjectileInputs() {
        const presetKey = DOMElements.projectileType.value;
        const preset = PROJECTILE_PRESETS[presetKey];
        if (!preset) return;

        DOMElements.projectileForm.elements['mass'].value = preset.mass;
        DOMElements.projectileForm.elements['dragCoefficient'].value = preset.dragCoefficient;
        DOMElements.projectileForm.elements['referenceArea'].value = preset.referenceArea;
    }

    // --- DATA DISPLAY ---
    function updateLiveSummary(point, maxAltitude, maxSpeed, maxMach = 0) {
        if (!point) return;

        const format = (num) => num.toLocaleString(undefined, { maximumFractionDigits: 1 });

        DOMElements.summary.maxAltitude.textContent = format(maxAltitude);
        DOMElements.summary.range.textContent = format(point.x);
        DOMElements.summary.flightTime.textContent = format(point.time);
        DOMElements.summary.maxSpeed.textContent = format(maxSpeed);

        if (state.simulationType === 'rocket') {
            DOMElements.summary.maxMach.textContent = maxMach.toFixed(2);
            // Burnout altitude is handled by the final updateSummary call
        }
    }


    function updateSummary(result) {
        const data = result.data;
        if (!data || data.length === 0) return;

        const lastPoint = data[data.length - 1] || { x: 0, time: 0 };
        const maxAltitude = Math.max(...data.map(p => p.y));
        const maxSpeed = Math.max(...data.map(p => p.speed));

        const format = (num) => num.toLocaleString(undefined, { maximumFractionDigits: 1 });

        DOMElements.summary.maxAltitude.textContent = format(maxAltitude);
        DOMElements.summary.range.textContent = format(lastPoint.x);
        DOMElements.summary.flightTime.textContent = format(lastPoint.time);
        DOMElements.summary.maxSpeed.textContent = format(maxSpeed);

        if (state.simulationType === 'rocket') {
            const maxMach = Math.max(...data.map(p => p.mach));
            DOMElements.summary.maxMach.textContent = maxMach.toFixed(2);
        }
    }

    function updateCharts(data, isFinalUpdate = false) {
        if (!data) return;

        const gridColor = state.isDarkMode ? 'rgba(255, 255, 255, 0.1)' : 'rgba(0, 0, 0, 0.1)';
        const textColor = state.isDarkMode ? '#f4f7fb' : '#172033';
        Chart.defaults.color = textColor;
        Chart.defaults.borderColor = gridColor;

        const commonOptions = {
            responsive: true,
            maintainAspectRatio: false,
            interaction: { mode: 'index', intersect: false },
            animation: {
                duration: 0 // Disable default chart animations
            },
            plugins: {
                legend: { position: 'top' },
                zoom: {
                    pan: { 
                        enabled: true, 
                        mode: 'xy',
                        onPanComplete: redrawOverlay
                    },
                    zoom: { 
                        wheel: { enabled: true }, pinch: { enabled: true }, mode: 'xy',
                        onZoomComplete: redrawOverlay
                    }
                }
            },
        };

        // Sync animation canvas size with chart canvas
        DOMElements.animationCanvas.width = DOMElements.trajectoryChartCanvas.clientWidth;
        DOMElements.animationCanvas.height = DOMElements.trajectoryChartCanvas.clientHeight;
        DOMElements.animationCanvas.style.position = 'absolute';
        DOMElements.animationCanvas.style.inset = '0';
        DOMElements.animationCanvas.style.pointerEvents = 'none';
        if (state.target) drawTarget();

        // Trajectory Chart
        createOrUpdateChart('trajectory', 'trajectory-chart', {
            ...chartConfigs.trajectory,
            options: { ...commonOptions, ...chartConfigs.trajectory.options },
            data: {
                datasets: [{
                    label: 'Trajectory',
                    // On initial call for projectile, data is empty for animation.
                // On final update, populate it fully to ensure it's all there.
                    data: (state.simulationType === 'projectile' && !isFinalUpdate) ? [] : data.map(p => ({ x: p.x, y: p.y })),
                    borderColor: chartColors.blue,
                    backgroundColor: chartColors.blue,
                    showLine: true,
                    pointRadius: 0,
                    tension: 0.1
                }]
            }
        });

        // Velocity Chart
        createOrUpdateChart('velocity', 'velocity-chart', {
            ...chartConfigs.velocity,
            options: { ...commonOptions, ...chartConfigs.velocity.options },
            data: {
                labels: (isFinalUpdate) ? data.map(p => p.time.toFixed(1)) : [],
                datasets: [
                    {
                        label: 'Total Speed',
                        data: data.map(p => p.speed),
                        borderColor: chartColors.blue,
                        pointRadius: 0,
                        tension: 0.1
                    },
                    {
                        label: 'Vx',
                        data: data.map(p => p.vx),
                        borderColor: chartColors.sky,
                        pointRadius: 0,
                        tension: 0.1,
                        hidden: true
                    },
                    {
                        label: 'Vy',
                        data: data.map(p => p.vy),
                        borderColor: chartColors.green,
                        pointRadius: 0,
                        tension: 0.1,
                        hidden: true
                    }
                ]
            }
        });

        // Altitude Chart
        createOrUpdateChart('altitude', 'altitude-chart', {
            ...chartConfigs.altitude,
            options: { ...commonOptions, ...chartConfigs.altitude.options },
            data: {
                labels: (isFinalUpdate) ? data.map(p => p.time.toFixed(1)) : [],
                datasets: [{
                    label: 'Altitude',
                    data: data.map(p => p.y),
                    borderColor: chartColors.green,
                    backgroundColor: state.isDarkMode ? 'rgba(22, 121, 79, 0.3)' : 'rgba(22, 121, 79, 0.1)',
                    fill: 'start',
                    pointRadius: 0,
                    tension: 0.1
                }]
            }
        });

        if (state.simulationType === 'rocket') {
            // Mass Chart
            createOrUpdateChart('mass', 'mass-chart', {
                ...chartConfigs.mass,
                options: { ...commonOptions, ...chartConfigs.mass.options },
                data: {
                    labels: (isFinalUpdate) ? data.map(p => p.time.toFixed(1)) : [],
                    datasets: [{
                        label: 'Mass',
                        data: data.map(p => p.mass),
                        borderColor: chartColors.orange,
                        pointRadius: 0,
                        tension: 0.1
                    }]
                }
            });

            // Mach & Cd Chart
            createOrUpdateChart('mach', 'mach-chart', {
                ...chartConfigs.mach,
                options: {
                    ...commonOptions, ...chartConfigs.mach.options,
                    scales: {
                        x: { title: { display: true, text: 'Time (s)' } },
                        y: { type: 'linear', position: 'left', title: { display: true, text: 'Mach Number' } },
                        y1: { type: 'linear', position: 'right', title: { display: true, text: 'Drag Coeff (Cd)' }, grid: { drawOnChartArea: false } }
                    }
                },
                data: {
                    labels: (isFinalUpdate) ? data.map(p => p.time.toFixed(1)) : [],
                    datasets: [
                        {
                            label: 'Mach',
                            data: data.map(p => p.mach),
                            borderColor: chartColors.red,
                            yAxisID: 'y',
                            pointRadius: 0,
                            tension: 0.1
                        },
                        {
                            label: 'Cd',
                            data: data.map(p => p.cd),
                            borderColor: chartColors.sky,
                            yAxisID: 'y1',
                            pointRadius: 0,
                            tension: 0.1
                        }
                    ]
                }
            });
        }
    }

    function createOrUpdateChart(id, canvasId, config) {
        if (state.charts[id]) {
            state.charts[id].data = config.data;
            state.charts[id].update();
        } else {
            const ctx = document.getElementById(canvasId).getContext('2d');
            state.charts[id] = new Chart(ctx, config);
        }
    }

    function init3DScene(params = {}) {
        const canvas = DOMElements.threeCanvas;
        const width = canvas.clientWidth;
        const height = canvas.clientHeight;

        // Scene
        three.scene = new THREE.Scene();
        three.scene.background = new THREE.Color(state.isDarkMode ? 0x0f1420 : 0xeef3f8);

        // Camera
        three.camera = new THREE.PerspectiveCamera(60, width / height, 1, 1000000);
        three.camera.position.set(three.launchOriginX - 2000, 2000, 5000);
        three.camera.lookAt(three.launchOriginX, 0, 0);

        // Controls
        three.controls = new OrbitControls(three.camera, canvas);
        three.controls.target.set(three.launchOriginX, 0, 0);
        three.controls.enabled = true;
        three.controls.enablePan = true;
        three.controls.enableZoom = true;
        three.controls.enableDamping = true; // for smooth camera motion
        three.controls.dampingFactor = 0.05;

        // Renderer
        three.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
        three.renderer.setSize(width, height);
        three.renderer.setPixelRatio(window.devicePixelRatio);

        // Lighting
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
        three.scene.add(ambientLight);
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(-1, 1, 1);
        three.scene.add(directionalLight);

        // Terrain
        const groundSize = 60000;
        const segments = 100;
        const groundGeometry = new THREE.PlaneGeometry(groundSize, groundSize, segments, segments);
        const groundMaterial = new THREE.MeshStandardMaterial({ color: 0x228B22, side: THREE.DoubleSide });
        three.ground = new THREE.Mesh(groundGeometry, groundMaterial);

        // Generate Hills
        const vertices = groundGeometry.attributes.position;
        for (let i = 0; i < vertices.count; i++) {
            const x = vertices.getX(i);
            const y = vertices.getY(i);
            const height = 1500 * (Math.sin(x / 5000) * Math.cos(y / 5000));
            vertices.setZ(i, height); // Displace the Z vertex (which becomes Y after rotation)
        }
        groundGeometry.computeVertexNormals(); // Recalculate normals for correct lighting

        three.ground.rotation.x = -Math.PI / 2;
        // We shift the ground down slightly so the projectile starts just above the average ground level
        three.ground.position.y = -500;
        three.scene.add(three.ground);
        three.ground.updateMatrixWorld(true);

        // Add simple landmarks to make the terrain easier to read at a glance.
        three.scenery = new THREE.Group();
        three.scene.add(three.scenery);
        const isRocket = state.simulationType === 'rocket';
        const landmarkScale = isRocket ? ROCKET_DISPLAY_SCALE : 1;

        const foliageMaterial = new THREE.MeshStandardMaterial({ color: 0x2f7d4a, roughness: 0.9 });
        const trunkMaterial = new THREE.MeshStandardMaterial({ color: 0x765033, roughness: 1 });
        const wallMaterial = new THREE.MeshStandardMaterial({ color: 0xd9b38c, roughness: 0.85 });
        const roofMaterial = new THREE.MeshStandardMaterial({ color: 0x9b3d32, roughness: 0.8 });
        const targetMaterial = new THREE.MeshStandardMaterial({ color: 0xf2d14b, roughness: 0.7 });
        const targetRingMaterial = new THREE.MeshStandardMaterial({ color: 0xc83e3e, roughness: 0.7 });

        const addForest = (candidateCount = isRocket ? 60000 : 3600) => {
            const placements = [];
            let seed = 42817;
            const random = () => {
                seed = (seed * 1664525 + 1013904223) >>> 0;
                return seed / 4294967296;
            };

            for (let i = 0; i < candidateCount; i++) {
                const x = THREE.MathUtils.randFloat(isRocket ? -30000 : -24000, isRocket ? 30000 : 24000);
                const z = THREE.MathUtils.randFloat(isRocket ? -9000 : -7000, isRocket ? 9000 : 7000);
                const clusterShape = 0.5 + 0.5 * Math.sin(x / 3600 + Math.sin(z / 2800)) * Math.cos(z / 3000);
                const density = isRocket ? 0.45 + clusterShape * 0.55 : 0.18 + clusterShape * 0.82;

                if (random() > density) continue;

                placements.push({
                    x,
                    z,
                    scale: THREE.MathUtils.randFloat(0.7, 1.3),
                    rotation: random() * Math.PI * 2,
                });
            }

            const trunkGeometry = new THREE.CylinderGeometry(24 * landmarkScale, 32 * landmarkScale, 180 * landmarkScale, 8);
            const crownGeometry = new THREE.ConeGeometry(125 * landmarkScale, 300 * landmarkScale, 8);
            const trunks = new THREE.InstancedMesh(trunkGeometry, trunkMaterial, placements.length);
            const crowns = new THREE.InstancedMesh(crownGeometry, foliageMaterial, placements.length);
            const dummy = new THREE.Object3D();

            placements.forEach(({ x, z, scale, rotation }, index) => {
                const terrainY = getTerrainHeight(x, z);
                dummy.scale.setScalar(scale);
                dummy.rotation.set(0, rotation, 0);

                dummy.position.set(x, terrainY + 90 * scale * landmarkScale, z);
                dummy.updateMatrix();
                trunks.setMatrixAt(index, dummy.matrix);

                dummy.position.set(x, terrainY + 290 * scale * landmarkScale, z);
                dummy.updateMatrix();
                crowns.setMatrixAt(index, dummy.matrix);
            });

            trunks.instanceMatrix.needsUpdate = true;
            crowns.instanceMatrix.needsUpdate = true;
            trunks.computeBoundingSphere();
            crowns.computeBoundingSphere();
            three.scenery.add(trunks, crowns);
        };

        const addHouses = () => {
            const placements = [];
            let seed = 17291;
            const random = () => {
                seed = (seed * 1664525 + 1013904223) >>> 0;
                return seed / 4294967296;
            };

            const clusterCount = isRocket ? 12 : 3 + Math.floor(random() * 3);
            const clusterCenters = [];

            while (clusterCenters.length < clusterCount) {
                const x = THREE.MathUtils.randFloat(isRocket ? -28000 : -20000, isRocket ? 28000 : 20000);
                const z = THREE.MathUtils.randFloat(isRocket ? -7500 : -8000, isRocket ? 7500 : 8000);
                const isTooClose = clusterCenters.some((center) => {
                    const distanceX = center.x - x;
                    const distanceZ = center.z - z;
                    return distanceX * distanceX + distanceZ * distanceZ < (isRocket ? 4500 : 7000) ** 2;
                });

                if (!isTooClose) clusterCenters.push({ x, z });
            }

            clusterCenters.forEach(({ x: centerX, z: centerZ }) => {
                const houseCount = isRocket ? 8 + Math.floor(random() * 10) : 5 + Math.floor(random() * 6);
                for (let i = 0; i < houseCount; i++) {
                    const angle = random() * Math.PI * 2;
                    const distance = THREE.MathUtils.randFloat(450, 1800);
                    const x = centerX + Math.cos(angle) * distance;
                    const z = centerZ + Math.sin(angle) * distance;

                    placements.push({
                        x,
                        z,
                        scale: THREE.MathUtils.randFloat(0.8, 1.15),
                        rotation: random() * Math.PI * 2,
                    });
                }
            });

            const walls = new THREE.InstancedMesh(
                new THREE.BoxGeometry(420 * landmarkScale, 260 * landmarkScale, 360 * landmarkScale),
                wallMaterial,
                placements.length
            );
            const roofs = new THREE.InstancedMesh(
                new THREE.ConeGeometry(310 * landmarkScale, 230 * landmarkScale, 4),
                roofMaterial,
                placements.length
            );
            const dummy = new THREE.Object3D();

            placements.forEach(({ x, z, scale, rotation }, index) => {
                const terrainY = getTerrainHeight(x, z);
                dummy.scale.setScalar(scale);
                dummy.rotation.set(0, rotation, 0);

                dummy.position.set(x, terrainY + 130 * scale * landmarkScale, z);
                dummy.updateMatrix();
                walls.setMatrixAt(index, dummy.matrix);

                dummy.rotation.y = rotation + Math.PI / 4;
                dummy.position.set(x, terrainY + 375 * scale * landmarkScale, z);
                dummy.updateMatrix();
                roofs.setMatrixAt(index, dummy.matrix);
            });

            walls.instanceMatrix.needsUpdate = true;
            roofs.instanceMatrix.needsUpdate = true;
            walls.computeBoundingSphere();
            roofs.computeBoundingSphere();
            three.scenery.add(walls, roofs);
        };

        const addTarget = (x, z) => {
            const target = new THREE.Group();
            const pole = new THREE.Mesh(new THREE.CylinderGeometry(14, 14, 520, 8), trunkMaterial);
            pole.position.y = 260;
            target.add(pole);

            const board = new THREE.Mesh(new THREE.CylinderGeometry(170, 170, 24, 32), targetMaterial);
            board.rotation.x = Math.PI / 2;
            board.position.y = 430;
            target.add(board);

            const ring = new THREE.Mesh(new THREE.TorusGeometry(90, 18, 8, 32), targetRingMaterial);
            ring.position.y = 430;
            target.add(ring);

            target.position.set(x, getTerrainHeight(x, z), z);
            three.scenery.add(target);
        };

        addForest();

        addHouses();

        addTarget(12000, 0);
        addTarget(24500, 1800);
        addTarget(-16500, -1200);

        // Projectile or rocket marker
        const projectileGeometry = isRocket
            ? new THREE.ConeGeometry(120 * ROCKET_DISPLAY_SCALE, 420 * ROCKET_DISPLAY_SCALE, 12)
            : new THREE.SphereGeometry(150, 16, 16);
        const projectileMaterial = new THREE.MeshStandardMaterial({ color: isRocket ? 0xd6d9df : 0xff6347 });
        three.projectile = new THREE.Mesh(projectileGeometry, projectileMaterial);
        three.projectile.visible = true; // Ensure it's visible on new runs
        three.scene.add(three.projectile);
        
        if (isRocket) {
            const launchPadMaterial = new THREE.MeshStandardMaterial({ color: 0x4a5568, roughness: 0.8 });
            const launchPad = new THREE.Mesh(new THREE.BoxGeometry(
                1800 * ROCKET_DISPLAY_SCALE,
                180 * ROCKET_DISPLAY_SCALE,
                1800 * ROCKET_DISPLAY_SCALE
            ), launchPadMaterial);
            launchPad.position.set(three.launchOriginX, 0, 0);
            three.scene.add(launchPad);
            snapToTerrain(launchPad, three.ground);
        } else {
        // Artillery Model (Field Gun Style)
        const artillery = new THREE.Group();

        // Materials
        const greenMat = new THREE.MeshStandardMaterial({ color: 0x3b4d28, roughness: 0.7 });
        const darkMat = new THREE.MeshStandardMaterial({ color: 0x1a1a1a, roughness: 0.9 });
        const metalMat = new THREE.MeshStandardMaterial({ color: 0x444444, metalness: 0.6, roughness: 0.3 });

        // 1. Base / Carriage (Sloped trail legs)
        const baseGeom = new THREE.BoxGeometry(0.6, 0.3, 2.0);
        const base = new THREE.Mesh(baseGeom, greenMat);
        base.position.set(0, 0.4, 0.5); // Extended backward along Z-axis
        artillery.add(base);

        // 2. Wheels (Z-axis alignment fix)
        const wheelGeom = new THREE.CylinderGeometry(0.6, 0.6, 0.2, 24);
        wheelGeom.rotateZ(Math.PI / 2); // Rotated around Z so flat faces point out left/right

        const leftWheel = new THREE.Mesh(wheelGeom, darkMat);
        leftWheel.position.set(-0.5, 0.6, 0);
        artillery.add(leftWheel);

        const rightWheel = new THREE.Mesh(wheelGeom, darkMat);
        rightWheel.position.set(0.5, 0.6, 0);
        artillery.add(rightWheel);

        // 3. Axle
        const axleGeom = new THREE.CylinderGeometry(0.08, 0.08, 1.2, 8);
        axleGeom.rotateZ(Math.PI / 2); 
        const axle = new THREE.Mesh(axleGeom, metalMat);
        axle.position.set(0, 0.6, 0);
        artillery.add(axle);

        // 4. Barrel Pivot (Placed directly on the axle line)
        const barrelPivot = new THREE.Group();
        barrelPivot.position.set(0, 0.7, 0); 
        artillery.add(barrelPivot);

        // 5. Barrel (Tapered and rotated to point FORWARD along negative Z-axis)
        // Top radius: 0.1, Bottom radius: 0.18, Length: 2.2
        const barrelGeom = new THREE.CylinderGeometry(0.1, 0.18, 2.2, 16);
        barrelGeom.rotateX(Math.PI / 2); // Crucial: Rotates the vertical cylinder to point along the Z-axis
        const barrel = new THREE.Mesh(barrelGeom, metalMat);

        // Offset the barrel mesh forward so it pivots near the breech (back end)
        barrel.position.set(0, 0, -0.8); 
        barrelPivot.add(barrel);

        // Optional: Give it a slight upward firing angle default (e.g., 15 degrees)
        barrelPivot.rotation.x = -Math.PI / 12; 

        // Scale and position the entire model
        artillery.scale.set(250, 250, 250);
        artillery.position.set(0, 0, 0);
        three.scene.add(artillery);
        snapToTerrain(artillery, three.ground);
        }

        three.windIndicator = createWindIndicator(params.windSpeed ?? 0, params.windDirection ?? 0);
        three.scene.add(three.windIndicator);

        // Trajectory Line
        const lineMaterial = new THREE.LineBasicMaterial({ color: 0x2155cd });
        const lineGeometry = new THREE.BufferGeometry();
        three.line = new THREE.Line(lineGeometry, lineMaterial);
        three.scene.add(three.line);
    }

    function createWindIndicator(windSpeed, windDirection) {
        const directionRadians = THREE.MathUtils.degToRad(windDirection);
        const direction = new THREE.Vector3(Math.cos(directionRadians), 0, Math.sin(directionRadians)).normalize();
        const arrowLength = 700 + Math.min(windSpeed, 500) * 28;
        const indicator = new THREE.Group();
        const arrow = new THREE.ArrowHelper(direction, new THREE.Vector3(0, 0, 0), arrowLength, 0x23b5d3, 260, 130);
        indicator.add(arrow);

        const labelCanvas = document.createElement('canvas');
        labelCanvas.width = 720;
        labelCanvas.height = 150;
        const context = labelCanvas.getContext('2d');
        context.fillStyle = 'rgba(15, 20, 32, 0.82)';
        context.fillRect(0, 0, labelCanvas.width, labelCanvas.height);
        context.fillStyle = '#8de8f5';
        context.font = 'bold 42px sans-serif';
        context.textAlign = 'center';
        context.textBaseline = 'middle';
        context.fillText(`Wind ${windSpeed.toFixed(1)} m/s | ${windDirection.toFixed(0)}°`, labelCanvas.width / 2, labelCanvas.height / 2);

        const labelTexture = new THREE.CanvasTexture(labelCanvas);
        const labelMaterial = new THREE.SpriteMaterial({ map: labelTexture, transparent: true, depthTest: false });
        const label = new THREE.Sprite(labelMaterial);
        label.position.set(0, 500, 0);
        label.scale.set(2600, 540, 1);
        indicator.add(label);
        indicator.userData.resources = { labelTexture, labelMaterial };
        indicator.position.set(three.launchOriginX, three.launchElevation + 1100, 0);
        return indicator;
    }

    function disposeWindIndicator() {
        if (!three.windIndicator) return;

        const resources = three.windIndicator.userData.resources;
        if (resources) {
            resources.labelTexture.dispose();
            resources.labelMaterial.dispose();
        }
        three.scene?.remove(three.windIndicator);
        three.windIndicator = null;
    }

    function show3DTargetMarker(position) {
        dispose3DTargetMarker();

        const marker = new THREE.Group();
        const markerMaterial = new THREE.MeshStandardMaterial({
            color: 0xff3b30,
            emissive: 0x5a0804,
            emissiveIntensity: 1.5,
            roughness: 0.35
        });
        const ring = new THREE.Mesh(new THREE.TorusGeometry(260, 34, 12, 32), markerMaterial);
        ring.rotation.x = Math.PI / 2;
        ring.position.y = 35;
        marker.add(ring);

        const beacon = new THREE.Mesh(new THREE.CylinderGeometry(24, 24, 700, 12), markerMaterial);
        beacon.position.y = 350;
        marker.add(beacon);

        const cap = new THREE.Mesh(new THREE.SphereGeometry(70, 16, 12), markerMaterial);
        cap.position.y = 720;
        marker.add(cap);

        marker.position.copy(position);
        marker.userData.resources = { markerMaterial };
        three.scene.add(marker);
        three.targetMarker = marker;
        three.renderer.render(three.scene, three.camera);
    }

    function dispose3DTargetMarker() {
        if (!three.targetMarker) return;

        const resources = three.targetMarker.userData.resources;
        if (resources?.markerMaterial) resources.markerMaterial.dispose();
        three.targetMarker.traverse((object) => {
            if (object.geometry) object.geometry.dispose();
        });
        three.scene?.remove(three.targetMarker);
        three.targetMarker = null;
    }

    function snapToTerrain(object, terrain) {
        const raycaster = new THREE.Raycaster();
        const rayOrigin = new THREE.Vector3(object.position.x, 10000, object.position.z);
        const downDirection = new THREE.Vector3(0, -1, 0);

        raycaster.set(rayOrigin, downDirection);
        const intersections = raycaster.intersectObject(terrain, false);
        if (intersections.length === 0) return;

        const hit = intersections[0];
        const surfaceNormal = hit.face.normal.clone().transformDirection(terrain.matrixWorld);
        const upVector = new THREE.Vector3(0, 1, 0);

        object.position.copy(hit.point);
        object.quaternion.setFromUnitVectors(upVector, surfaceNormal);
        three.launchElevation = hit.point.y;
    }

    function findTerrainCollision(previousPosition, currentPosition) {
        const travel = new THREE.Vector3().subVectors(currentPosition, previousPosition);
        const travelDistance = travel.length();
        if (travelDistance <= 1e-6) return null;

        const raycaster = new THREE.Raycaster(
            previousPosition,
            travel.normalize(),
            0,
            travelDistance
        );
        const intersections = raycaster.intersectObject(three.ground, true);
        return intersections.length > 0 ? intersections[0].point : null;
    }

    function update3DScene(frameData, points, worldLaunchElevation = 0, worldScale = 1, launchOriginX = 0) {
        if (!three.renderer) return;

        // Update projectile position
        const worldX = launchOriginX + frameData.x * worldScale;
        const worldY = frameData.y * worldScale + worldLaunchElevation;
        const worldZ = (frameData.z ?? 0) * worldScale;
        three.projectile.position.set(worldX, worldY, worldZ);

        if (three.windIndicator) {
            three.windIndicator.position.set(worldX, worldY + 1100, 0);
        }

        if (state.simulationType === 'rocket') {
            const flightDirection = new THREE.Vector3(
                frameData.vx ?? 0,
                frameData.vy ?? 0,
                frameData.vz ?? 0
            );
            if (flightDirection.lengthSq() > 1e-6) {
                // ConeGeometry points along local +Y; align it with the full 3D velocity vector.
                three.projectile.quaternion.setFromUnitVectors(
                    new THREE.Vector3(0, 1, 0),
                    flightDirection.normalize()
                );
            }
        }

        // Smoothly move the camera's target to follow the projectile
        const targetPosition = new THREE.Vector3(worldX, worldY, worldZ);
        three.controls.target.lerp(targetPosition, 0.1);
        three.controls.update();

        // Update trajectory line
        three.line.geometry.setFromPoints(points);

        // Render
        three.renderer.render(three.scene, three.camera);
    }

    /**
     * Creates a particle-based explosion effect at a given position.
     * @param {number} impactX - The world x-coordinate for the explosion.
     * @param {number} impactY - The world y-coordinate for the explosion.
     * @param {number} impactZ - The world z-coordinate for the explosion.
     * @returns {object} A reference to the created particle system and its properties.
     */
    function createBlast(impactX, impactY, impactZ) {
        if (!three.scene) return;
    
        const particleCount = 1000;
        const positions = new Float32Array(particleCount * 3);
        const velocities = [];
    
        // All particles spawn at the exact point of impact
        for (let i = 0; i < particleCount; i++) {
            positions[i * 3] = impactX;
            positions[i * 3 + 1] = impactY;
            positions[i * 3 + 2] = impactZ;
    
            // Spherical blast calculation direction vectors
            const theta = Math.random() * Math.PI * 2;
            const phi = Math.acos((Math.random() * 2) - 1);
            const speed = 50 + Math.random() * 150; // Random force dispersion
    
            velocities.push({
                x: Math.sin(phi) * Math.cos(theta) * speed,
                y: Math.sin(phi) * Math.sin(theta) * speed,
                z: Math.cos(phi) * speed
            });
        }
    
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    
        const material = new THREE.PointsMaterial({
            color: 0xff5500, // Fiery orange blast
            size: 35,
            transparent: true,
            opacity: 1.0,
            blending: THREE.AdditiveBlending, // Glow effect
            depthWrite: false,
        });
    
        const particleSystem = new THREE.Points(geometry, material);
        three.scene.add(particleSystem);
    
        // Return reference along with its unique speed parameters for animation
        return { mesh: particleSystem, geometry, material, velocities, count: particleCount };
    }

    function drawTarget() {
        if (!state.target || !state.charts.trajectory) return;
    
        const chart = state.charts.trajectory;
        const ctx = DOMElements.animationCtx;
        const chartArea = chart.chartArea;
        if (!chartArea) return;
    
        // Convert target data coordinates to pixel coordinates
        const xPixel = chart.scales.x.getPixelForValue(state.target.x);
        const yPixel = chart.scales.y.getPixelForValue(state.target.y);
    
        // Convert 100-unit radius to pixels. Average of X and Y scales for a circular appearance.
        const xPixelRadius = chart.scales.x.getPixelForValue(state.target.x + 100) - xPixel;
        const yPixelRadius = yPixel - chart.scales.y.getPixelForValue(state.target.y + 100);
        const pixelRadius = (xPixelRadius + yPixelRadius) / 2;
    
        // Draw hitbox circle
        ctx.beginPath();
        ctx.arc(xPixel, yPixel, pixelRadius, 0, 2 * Math.PI);
        ctx.fillStyle = 'rgba(255, 99, 132, 0.3)'; // Semi-transparent red
        ctx.strokeStyle = 'rgba(255, 99, 132, 0.8)';
        ctx.lineWidth = 2;
        ctx.fill();
        ctx.stroke();
    }

    function drawAnimatedProjectile(position, chart) {
        const ctx = DOMElements.animationCtx;
        const chartArea = chart.chartArea;
        if (!chartArea) return;

        // Convert data coordinates to canvas pixel coordinates
        const xPixel = chart.scales.x.getPixelForValue(position.x);
        const yPixel = chart.scales.y.getPixelForValue(position.y);

        // Clear previous frame
        ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);

        // Redraw the target so the projectile appears on top of it
        if (state.target) drawTarget();

        // Draw projectile if it's within the chart area
        if (xPixel >= chartArea.left && xPixel <= chartArea.right && yPixel >= chartArea.top && yPixel <= chartArea.bottom) {
            ctx.beginPath();
            ctx.arc(xPixel, yPixel, 5, 0, 2 * Math.PI);
            ctx.fillStyle = state.isDarkMode ? chartColors.orange : chartColors.red;
            ctx.fill();
        }
    }

    function updateTable(data) {
        if (!data) {
            resetUI();
            return;
        }

        updateTableHeaders();

        const limit = parseInt(DOMElements.table.limit.value, 10);
        const step = Math.max(1, Math.floor(data.length / limit));
        const sampledData = data.filter((_, i) => i % step === 0 || i === data.length - 1).slice(0, limit);

        const format = (num) => num.toLocaleString(undefined, { minimumFractionDigits: 2, maximumFractionDigits: 2 });

        let tableHTML = '';
        if (state.simulationType === 'projectile') {
            tableHTML = sampledData.map(p => `
                <tr>
                    <td>${format(p.time)}</td>
                    <td>${format(p.x)}</td>
                    <td>${format(p.y)}</td>
                    <td>${format(p.vx)}</td>
                    <td>${format(p.vy)}</td>
                    <td>${format(p.speed)}</td>
                </tr>
            `).join('');
        } else { // Rocket
            tableHTML = sampledData.map(p => `
                <tr>
                    <td>${format(p.time)}</td>
                    <td>${format(p.x)}</td>
                    <td>${format(p.y)}</td>
                    <td>${format(p.speed)}</td>
                    <td>${p.mach.toFixed(3)}</td>
                    <td>${p.cd.toFixed(3)}</td>
                    <td>${format(p.mass)}</td>
                </tr>
            `).join('');
        }

        DOMElements.table.body.innerHTML = tableHTML;
    }

    function updateTableHeaders() {
        let headers = [];
        if (state.simulationType === 'projectile') {
            headers = ['Time (s)', 'X (m)', 'Y (m)', 'Vx (m/s)', 'Vy (m/s)', 'Speed (m/s)'];
        } else {
            headers = ['Time (s)', 'X (m)', 'Y (m)', 'Speed (m/s)', 'Mach', 'Cd', 'Mass (kg)'];
        }
        DOMElements.table.head.innerHTML = headers.map(h => `<th scope="col">${h}</th>`).join('');
    }

    // --- EXPORT ---
    function exportToCSV() {
        if (!state.simulationData) return;

        const data = state.simulationData;
        const headers = Object.keys(data[0]);
        const csvRows = [
            headers.join(','),
            ...data.map(row => headers.map(header => row[header]).join(','))
        ];

        const csvString = csvRows.join('\n');
        const blob = new Blob([csvString], { type: 'text/csv' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `${state.simulationType}_trajectory_${new Date().toISOString()}.csv`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    }

    // --- START THE APP ---
    init();
});