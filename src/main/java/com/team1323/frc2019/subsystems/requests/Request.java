package com.team1323.frc2019.subsystems.requests;

import java.util.ArrayList;
import java.util.List;

public abstract class Request {
	
	public abstract void act();
	
	public boolean isFinished(){return true;}

	public List<Prerequisite> prerequisites = new ArrayList<>();

	public void withPrerequisites(List<Prerequisite> reqs){
		for(Prerequisite req : reqs){
			prerequisites.add(req);
		}
	}

	public void withPrerequisite(Prerequisite req){
		prerequisites.add(req);
	}

	public boolean allowed(){
		boolean reqsMet = true;
		for(Prerequisite req : prerequisites){
			reqsMet &= req.met();
		}
		return reqsMet;
	}
	
}
