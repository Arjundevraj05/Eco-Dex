'use client';
import { useState } from "react";

const isDemoMode = process.env.NEXT_PUBLIC_DEMO_MODE === "true";

export default function OverrideControl() {
    const [status, setStatus] = useState("IDLE");

    const sendCommand = async (command: string) => {
        if (isDemoMode) {
            setStatus(command);
            return;
        }

        const res = await fetch("/api/override", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ command }),
        });

        const data = await res.json();
        if (data.status) setStatus(data.status);
    };

    return (
        <div className="p-5 bg-white shadow-lg rounded-lg text-center">
            <h2 className="text-2xl font-bold mb-4">Security Override</h2>
            {isDemoMode && (
                <p className="text-sm text-gray-500 mb-2">
                    Simulated control panel — buttons update status locally in demo mode
                </p>
            )}
            <p className="text-lg font-semibold">Status: <span className="text-green-600">{status}</span></p>

            <div className="mt-4 flex justify-center gap-4">
                <button onClick={() => sendCommand("LOCK")} className="px-4 py-2 bg-red-500 text-white rounded-lg hover:bg-red-600 transition-colors">Lock</button>
                <button onClick={() => sendCommand("UNLOCK")} className="px-4 py-2 bg-green-500 text-white rounded-lg hover:bg-green-600 transition-colors">Unlock</button>
                <button onClick={() => sendCommand("ALERT")} className="px-4 py-2 bg-yellow-500 text-white rounded-lg hover:bg-yellow-600 transition-colors">Alert</button>
            </div>
        </div>
    );
}
