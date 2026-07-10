"use client";

import { useEffect, useState } from "react";
import { FaRobot, FaTimes } from "react-icons/fa";

const STORAGE_KEY = "ecodex-demo-modal-dismissed";

export default function DemoModal() {
  const [isOpen, setIsOpen] = useState(false);

  useEffect(() => {
    if (process.env.NEXT_PUBLIC_DEMO_MODE !== "true") return;

    const dismissed = sessionStorage.getItem(STORAGE_KEY);
    if (!dismissed) {
      setIsOpen(true);
    }
  }, []);

  const handleClose = () => {
    sessionStorage.setItem(STORAGE_KEY, "true");
    setIsOpen(false);
  };

  if (!isOpen || process.env.NEXT_PUBLIC_DEMO_MODE !== "true") {
    return null;
  }

  return (
    <div className="fixed inset-0 z-[1200] flex items-center justify-center p-4">
      <div
        className="absolute inset-0 bg-black/50 backdrop-blur-sm"
        onClick={handleClose}
        aria-hidden="true"
      />

      <div
        role="dialog"
        aria-modal="true"
        aria-labelledby="demo-modal-title"
        className="relative flex max-h-[90vh] w-full max-w-lg flex-col overflow-hidden rounded-2xl bg-white shadow-2xl"
      >
        <div className="bg-gradient-to-r from-green-600 to-green-700 px-6 py-5 text-white">
          <div className="flex items-start justify-between gap-4">
            <div className="flex items-center gap-3">
              <div className="rounded-full bg-white/20 p-3">
                <FaRobot className="h-6 w-6" />
              </div>
              <div>
                <p className="text-sm font-medium text-green-100 uppercase tracking-wide">
                  Demo Mode
                </p>
                <h2 id="demo-modal-title" className="text-xl font-bold">
                  Welcome to Eco-Dex
                </h2>
              </div>
            </div>
            <button
              onClick={handleClose}
              className="rounded-full p-1 hover:bg-white/20 transition-colors"
              aria-label="Close demo notice"
            >
              <FaTimes className="h-5 w-5" />
            </button>
          </div>
        </div>

        <div className="space-y-4 overflow-y-auto px-6 py-6 text-gray-700">
          <p className="leading-relaxed">
            You are viewing an <strong>interactive demo</strong> of the Eco-Dex dashboard.
            The RAG-ED robots are <strong>not live</strong> right now — all waste collection
            stats, map locations, and reports use <strong>simulated sample data</strong>.
          </p>

          <ul className="space-y-2 text-sm">
            <li className="flex items-start gap-2">
              <span className="mt-1.5 h-1.5 w-1.5 rounded-full bg-green-500 shrink-0" />
              Dashboard, reports, and map reflect demo collections
            </li>
            <li className="flex items-start gap-2">
              <span className="mt-1.5 h-1.5 w-1.5 rounded-full bg-green-500 shrink-0" />
              Live camera and override controls are simulated locally
            </li>
            <li className="flex items-start gap-2">
              <span className="mt-1.5 h-1.5 w-1.5 rounded-full bg-green-500 shrink-0" />
              No login or hardware connection is required
            </li>
          </ul>

          <button
            onClick={handleClose}
            className="w-full mt-2 rounded-lg bg-green-600 py-3 font-semibold text-white hover:bg-green-700 transition-colors"
          >
            Explore the Demo
          </button>
        </div>
      </div>
    </div>
  );
}
