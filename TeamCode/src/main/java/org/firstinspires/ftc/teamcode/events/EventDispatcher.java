package org.firstinspires.ftc.teamcode.events;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class EventDispatcher {
    private Map<String, ArrayList<Listener>> listeners = new HashMap<>();

    public void addEventListener(String eventName, Listener listener) {
        if(!listeners.containsKey(eventName)) listeners.put(eventName, new ArrayList<>());

        ArrayList<Listener> currentListeners = listeners.get(eventName);
        currentListeners.add(listener);
        listeners.put(eventName, currentListeners);
    }

    public void removeEventListener(String eventName, Listener listener) {
        if(!listeners.containsKey(eventName)) return;

        ArrayList<Listener> currentListeners = listeners.get(eventName);
        currentListeners.remove(listener);
        listeners.put(eventName, currentListeners);
    }

    protected void dispatchEvent(String eventName, String[] args) {
        if(!listeners.containsKey(eventName)) return;

        listeners.get(eventName).forEach(listener -> {
            listener.onEvent(eventName, args);
        });
    }
}
