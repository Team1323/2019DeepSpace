/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.subsystems.requests;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A Request which itself takes a list of Requests and executes them in series.
 */
public class SequentialRequest extends Request {

    List<Request> requests;
    Request currentRequest = null;

    public SequentialRequest(Request... requests) {
        this.requests = new ArrayList<>();
        for(Request request : requests) {
            this.requests.add(request);
        }
    }

    public SequentialRequest(List<Request> requests) {
        this.requests = new ArrayList<>();
        for(Request request : requests) {
            this.requests.add(request);
        }
    }

    @Override
    public void act() {
        currentRequest = requests.remove(0);
        currentRequest.act();
    }

    @Override
    public boolean isFinished(){
        if(currentRequest.isFinished()) {
            if(requests.isEmpty()) {
                currentRequest = null;
                return true;
            } else {
                currentRequest = requests.remove(0);
                currentRequest.act();
            }
        }
        return false;
    }
}
