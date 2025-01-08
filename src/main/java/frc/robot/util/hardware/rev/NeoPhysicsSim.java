/*
 * Copyright (c) 2022 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package frc.robot.util.hardware.rev;
import java.util.ArrayList;
import edu.wpi.first.math.system.plant.DCMotor;

/** Manages physics simulation for REV Robotics products. */
public class NeoPhysicsSim {
    private static final NeoPhysicsSim sim = new NeoPhysicsSim();

    /** Gets the robot simulator instance. */
    public static NeoPhysicsSim getInstance() {
        return sim;
    }

    public void addNeo(Neo neo, final float stallTorque, final float freeSpeed) {
        if (neo != null) {
            NeoSimProfile simSpark = new NeoSimProfile(neo, stallTorque, freeSpeed);
            _simProfiles.add(simSpark);
        }
    }

    public void addNeo(Neo neo, final DCMotor motor) {
        if (neo != null) {
            NeoSimProfile simSpark = new NeoSimProfile(neo, motor);
            _simProfiles.add(simSpark);
        }
    }

    /**
     * Runs the simulator: - enable the robot - runs all SparkMax devices connected
     */
    public void run() {
        // Simulate devices
        for (SimProfile simProfile : _simProfiles) {
            simProfile.run();
        }
    }

    private final ArrayList<SimProfile> _simProfiles = new ArrayList<SimProfile>();

    /** Holds information about a simulated device. */
    static class SimProfile {
        private long _lastTime;
        private boolean _running = false;

        /** Runs the simulation profile. Implemented by device-specific profiles. */
        public void run() {
        }

        /** Returns the time since last call, in milliseconds. */
        protected double getPeriod() {
            // set the start time if not yet running
            if (!_running) {
                _lastTime = System.nanoTime();
                _running = true;
            }

            long now = System.nanoTime();
            final double period = (now - _lastTime) / 1000000.;
            _lastTime = now;

            return period;
        }
    }

}