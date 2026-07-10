'use client';

import { useState } from 'react';
import { motion } from 'framer-motion';
import { Lock, Unlock, Bell, Shield, AlertTriangle, CheckCircle } from 'lucide-react';
import { FadeIn, Stagger, StaggerItem } from '@/components/AnimatedSection';
import { tapScale } from '@/lib/motion';

const isDemoMode = process.env.NEXT_PUBLIC_DEMO_MODE === 'true';

export default function SecurityOverridePage() {
  return (
    <div className="min-h-screen bg-green-50 px-4 py-6 sm:px-6 sm:py-8">
      <div className="mx-auto max-w-3xl">
        <FadeIn>
        <div className="mb-6 rounded-2xl bg-gradient-to-r from-slate-800 to-slate-700 p-5 text-white shadow-lg sm:mb-8 sm:p-6">
          <div className="mb-1 flex items-center gap-3">
            <Shield className="h-6 w-6 text-green-400 sm:h-7 sm:w-7" />
            <span className="text-xs font-medium uppercase tracking-wider opacity-80 sm:text-sm">Safety System</span>
          </div>
          <h1 className="text-2xl font-bold sm:text-3xl">Security Override</h1>
          <p className="mt-1 text-sm text-slate-300">
            Emergency lock, unlock, and alert controls for the RAG-ED security subsystem
          </p>
        </div>
        </FadeIn>

        <Stagger className="mb-6 grid grid-cols-1 gap-4 md:grid-cols-3">
          <StaggerItem><InfoCard icon={<CheckCircle className="h-5 w-5 text-green-500" />} title="System" value="Online" /></StaggerItem>
          <StaggerItem><InfoCard icon={<Shield className="h-5 w-5 text-blue-500" />} title="Auth" value="Verified" /></StaggerItem>
          <StaggerItem><InfoCard icon={<AlertTriangle className="h-5 w-5 text-yellow-500" />} title="Alerts" value="None" /></StaggerItem>
        </Stagger>

        <FadeIn delay={0.15}>
        <EnhancedOverrideControl />
        </FadeIn>

        {isDemoMode && (
          <p className="text-center text-sm text-gray-400 mt-4">
            Demo mode — security commands are simulated locally
          </p>
        )}
      </div>
    </div>
  );
}

function InfoCard({ icon, title, value }: { icon: React.ReactNode; title: string; value: string }) {
  return (
    <div className="bg-white rounded-xl shadow-sm p-4 border border-gray-100 flex items-center gap-3">
      {icon}
      <div>
        <p className="text-xs text-gray-400">{title}</p>
        <p className="font-semibold text-gray-800">{value}</p>
      </div>
    </div>
  );
}

function EnhancedOverrideControl() {
  const [status, setStatus] = useState('IDLE');

  const sendCommand = async (command: string) => {
    if (isDemoMode) {
      setStatus(command);
      return;
    }
    const res = await fetch('/api/override', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ command }),
    });
    const data = await res.json();
    if (data.status) setStatus(data.status);
  };

  const statusConfig: Record<string, { bg: string; text: string; dot: string }> = {
    IDLE: { bg: 'bg-gray-100', text: 'text-gray-600', dot: 'bg-gray-400' },
    LOCK: { bg: 'bg-red-50', text: 'text-red-700', dot: 'bg-red-500' },
    UNLOCK: { bg: 'bg-green-50', text: 'text-green-700', dot: 'bg-green-500' },
    ALERT: { bg: 'bg-yellow-50', text: 'text-yellow-700', dot: 'bg-yellow-500' },
  };

  const cfg = statusConfig[status] || statusConfig.IDLE;

  return (
    <div className="rounded-2xl border border-gray-100 bg-white p-5 shadow-md sm:p-8">
      <div className="mb-6 text-center sm:mb-8">
        <div className={`inline-flex items-center gap-2 rounded-full px-4 py-2 text-sm font-bold sm:px-5 ${cfg.bg} ${cfg.text}`}>
          <span className={`h-2.5 w-2.5 animate-pulse rounded-full ${cfg.dot}`} />
          {status}
        </div>
      </div>

      <div className="grid grid-cols-1 gap-4 sm:grid-cols-3">
        <motion.button
          onClick={() => sendCommand('LOCK')}
          whileHover={{ scale: 1.03, y: -2 }}
          whileTap={tapScale}
          className="group flex flex-col items-center gap-3 rounded-2xl border-2 border-red-100 bg-red-50 p-5 transition-colors hover:border-red-300 hover:bg-red-100 sm:p-6"
        >
          <div className="p-3 rounded-xl bg-red-500 text-white group-hover:scale-110 transition-transform">
            <Lock className="h-7 w-7" />
          </div>
          <span className="font-semibold text-red-700">Lock</span>
          <span className="text-xs text-red-400">Halt all motors</span>
        </motion.button>

        <motion.button
          onClick={() => sendCommand('UNLOCK')}
          whileHover={{ scale: 1.03, y: -2 }}
          whileTap={tapScale}
          className="group flex flex-col items-center gap-3 rounded-2xl border-2 border-green-100 bg-green-50 p-5 transition-colors hover:border-green-300 hover:bg-green-100 sm:p-6"
        >
          <div className="p-3 rounded-xl bg-green-500 text-white group-hover:scale-110 transition-transform">
            <Unlock className="h-7 w-7" />
          </div>
          <span className="font-semibold text-green-700">Unlock</span>
          <span className="text-xs text-green-400">Resume operation</span>
        </motion.button>

        <motion.button
          onClick={() => sendCommand('ALERT')}
          whileHover={{ scale: 1.03, y: -2 }}
          whileTap={tapScale}
          className="group flex flex-col items-center gap-3 rounded-2xl border-2 border-yellow-100 bg-yellow-50 p-5 transition-colors hover:border-yellow-300 hover:bg-yellow-100 sm:p-6"
        >
          <div className="p-3 rounded-xl bg-yellow-500 text-white group-hover:scale-110 transition-transform">
            <Bell className="h-7 w-7" />
          </div>
          <span className="font-semibold text-yellow-700">Alert</span>
          <span className="text-xs text-yellow-500">Trigger alarm</span>
        </motion.button>
      </div>
    </div>
  );
}
