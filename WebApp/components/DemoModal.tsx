"use client";

import { useEffect, useState } from "react";
import { AnimatePresence, motion } from "framer-motion";
import { FaRobot, FaTimes } from "react-icons/fa";
import { scaleInVariants, fadeInVariants } from "@/lib/motion";

const STORAGE_KEY = "ecodex-demo-modal-dismissed";

export default function DemoModal() {
  const [isOpen, setIsOpen] = useState(false);

  useEffect(() => {
    if (process.env.NEXT_PUBLIC_DEMO_MODE !== "true") return;

    const dismissed = sessionStorage.getItem(STORAGE_KEY);
    if (!dismissed) {
      const timer = setTimeout(() => setIsOpen(true), 400);
      return () => clearTimeout(timer);
    }
  }, []);

  const handleClose = () => {
    sessionStorage.setItem(STORAGE_KEY, "true");
    setIsOpen(false);
  };

  if (process.env.NEXT_PUBLIC_DEMO_MODE !== "true") {
    return null;
  }

  return (
    <AnimatePresence>
      {isOpen && (
        <div className="fixed inset-0 z-[1200] flex items-center justify-center p-4">
          <motion.div
            className="absolute inset-0 bg-black/50 backdrop-blur-sm"
            onClick={handleClose}
            aria-hidden="true"
            initial="hidden"
            animate="visible"
            exit="hidden"
            variants={fadeInVariants}
            transition={{ duration: 0.25 }}
          />

          <motion.div
            role="dialog"
            aria-modal="true"
            aria-labelledby="demo-modal-title"
            className="relative flex max-h-[90vh] w-full max-w-lg flex-col overflow-hidden rounded-2xl bg-white shadow-2xl"
            initial="hidden"
            animate="visible"
            exit="hidden"
            variants={scaleInVariants}
            transition={{ duration: 0.35, ease: [0.22, 1, 0.36, 1] }}
          >
            <div className="bg-gradient-to-r from-green-600 to-green-700 px-6 py-5 text-white">
              <div className="flex items-start justify-between gap-4">
                <div className="flex items-center gap-3">
                  <motion.div
                    className="rounded-full bg-white/20 p-3"
                    animate={{ rotate: [0, 5, -5, 0] }}
                    transition={{ duration: 2, repeat: Infinity, repeatDelay: 3 }}
                  >
                    <FaRobot className="h-6 w-6" />
                  </motion.div>
                  <div>
                    <p className="text-sm font-medium uppercase tracking-wide text-green-100">
                      Demo Mode
                    </p>
                    <h2 id="demo-modal-title" className="text-xl font-bold">
                      Welcome to Eco-Dex
                    </h2>
                  </div>
                </div>
                <button
                  onClick={handleClose}
                  className="rounded-full p-1 transition-colors hover:bg-white/20"
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
                {[
                  'Dashboard, reports, and map reflect demo collections',
                  'Live camera and override controls are simulated locally',
                  'No login or hardware connection is required',
                ].map((text, i) => (
                  <motion.li
                    key={text}
                    className="flex items-start gap-2"
                    initial={{ opacity: 0, x: -10 }}
                    animate={{ opacity: 1, x: 0 }}
                    transition={{ delay: 0.2 + i * 0.08 }}
                  >
                    <span className="mt-1.5 h-1.5 w-1.5 shrink-0 rounded-full bg-green-500" />
                    {text}
                  </motion.li>
                ))}
              </ul>

              <motion.button
                onClick={handleClose}
                whileHover={{ scale: 1.02 }}
                whileTap={{ scale: 0.98 }}
                className="mt-2 w-full rounded-lg bg-green-600 py-3 font-semibold text-white transition-colors hover:bg-green-700"
              >
                Explore the Demo
              </motion.button>
            </div>
          </motion.div>
        </div>
      )}
    </AnimatePresence>
  );
}
