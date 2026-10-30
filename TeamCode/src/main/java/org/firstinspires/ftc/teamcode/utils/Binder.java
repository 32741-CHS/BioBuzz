package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Binder {
    private final Map<String, List<Runnable>> binds = new HashMap<>();

    /**
     * Binds an anonymous function to a key. Supports binding multiple callbacks to the same key.
     *
     * @param key The string id, is case-sensitive and will the one you put in the call method to
     *     later call the thing
     * @param callback The function that will be called when you do Binder.call(key)
     */
    public void bind(String key, Runnable callback) {
        // Automatically creates a new ArrayList if the key doesn't exist yet and store to the
        // hashmap, then adds the callback to the list
        binds.computeIfAbsent(key, k -> new ArrayList<>()).add(callback);
    }

    /**
     * Executes all anonymous functions registered under the given key. If the key does not exist,
     * it safely does nothing.
     *
     * @param key The key you defined in the bind method.
     */
    public void call(String key) {
        // Safely retrieves the list, defaulting to an empty list if the key is missing
        List<Runnable> callbacks = binds.getOrDefault(key, Collections.emptyList());

        // Loop through and fire each runnable sequentially
        assert callbacks != null; // This will never be false, but IDE insist.
        for (Runnable callback : callbacks) {
            callback.run();
        }
    }
}
