package com.team1323.lib.util;

import java.util.ArrayList;
import java.util.List;

public class VisionCriteria {
    public enum Criterion {
        HEADING(1), DISTANCE(12);

        int allowableUpdates;
        int successfulUpdates = 0;
        private Criterion(int allowableUpdates){
            this.allowableUpdates = allowableUpdates;
            successfulUpdates = 0;
        }
        public void addSuccessfulUpdate(){
            successfulUpdates++;
        }
        public void reset(){
            successfulUpdates = 0;
        }
    }

    private List<Criterion> activeCriteria = new ArrayList<>();

    public VisionCriteria() {
        for(Criterion criterion : Criterion.values()){
            activeCriteria.add(criterion);
        }
    }

    public boolean updateAllowed(Criterion criterion){
        for(Criterion c : activeCriteria){
            if(criterion == c) return c.successfulUpdates < c.allowableUpdates;
        }
        return false;
    }

    public void addSuccessfulUpdate(Criterion criterion){
        for(Criterion c : activeCriteria){
            if(criterion == c){
                c.addSuccessfulUpdate();
                System.out.println(c.toString() + " updated " + c.successfulUpdates + " times.");
            } 
        }
    }

    public int successfulUpdates(Criterion criterion){
        for(Criterion c : activeCriteria){
            if(criterion == c) return c.successfulUpdates;
        }
        return 0;
    }

    public void reset(){
        for(Criterion criterion : activeCriteria){
            criterion.reset();
        }
    }
}