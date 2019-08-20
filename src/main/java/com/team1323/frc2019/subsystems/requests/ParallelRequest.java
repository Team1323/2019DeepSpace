/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.subsystems.requests;

import java.util.ArrayList;
import java.util.List;

/**
 * A Request which takes a list of Requests and executes them in parallel.
 */
public class ParallelRequest extends Request {

    List<Request> requests;

    public ParallelRequest(Request... requests) {
        this.requests = new ArrayList<>();
        for(Request request : requests) {
            this.requests.add(request);
        }
    }

    public ParallelRequest(List<Request> requests) {
        this.requests = new ArrayList<>();
        for(Request request : requests) {
            this.requests.add(request);
        }
    }

    @Override
    public void act() {
        for(Request request : requests) {
            request.act();
        }
    }

    @Override
    public boolean isFinished() {
        boolean finished = true;
        for(Request request : requests) {
            finished &= request.isFinished();
        }
        return finished;
    }
}
