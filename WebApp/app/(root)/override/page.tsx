'use client';

import { useState } from 'react';
import { Lock, Unlock, Bell, Shield, AlertTriangle, CheckCircle } from 'lucide-react';

const isDemoMode = process.env.NEXT_PUBLIC_DEMO_MODE === 'true';

export default function SecurityOverridePage() {
  return (
    <div className="ml-40 min-h-screen bg-green-50 py-8 px-6">
      <div className="max-w-3xl mx-auto">
        <div className="mb-8 rounded-2xl bg-gradient-to-r from-slate-800 to-slate-700 p-6 text-white shadow-lg">
          <div className="flex items-center gap-3 mb-1">
            <Shield className="h-7 w-7 text-green-400" />
            <span className="text-sm font-medium uppercase tracking-wider opacity-80">Safety System</span>
          </div>
          <h1 className="text-3xl font-bold">Security Override</h1>
          <p className="mt-1 text-slate-300 text-sm">
            Emergency lock, unlock, and alert controls for the RAG-ED security subsystem
          </p>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mb-6">
          <InfoCard icon={<CheckCircle className="h-5 w-5 text-green-500" />} title="System" value="Online" />
          <InfoCard icon={<Shield className="h-5 w-5 text-blue-500" />} title="Auth" value="Verified" />
          <InfoCard icon={<AlertTriangle className="h-5 w-5 text-yellow-500" />} title="Alerts" value="None" />
        </div>

        <EnhancedOverrideControl />

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
    <div className="bg-white rounded-2xl shadow-md p-8 border border-gray-100">
      <div className="text-center mb-8">
        <div className={`inline-flex items-center gap-2 rounded-full px-5 py-2 text-sm font-bold ${cfg.bg} ${cfg.text}`}>
          <span className={`w-2.5 h-2.5 rounded-full ${cfg.dot} animate-pulse`} />
          {status}
        </div>
      </div>

      <div className="grid grid-cols-3 gap-4">
        <button
          onClick={() => sendCommand('LOCK')}
          className="group flex flex-col items-center gap-3 p-6 rounded-2xl bg-red-50 border-2 border-red-100 hover:border-red-300 hover:bg-red-100 transition-all"
        >
          <div className="p-3 rounded-xl bg-red-500 text-white group-hover:scale-110 transition-transform">
            <Lock className="h-7 w-7" />
          </div>
          <span className="font-semibold text-red-700">Lock</span>
          <span className="text-xs text-red-400">Halt all motors</span>
        </button>

        <button
          onClick={() => sendCommand('UNLOCK')}
          className="group flex flex-col items-center gap-3 p-6 rounded-2xl bg-green-50 border-2 border-green-100 hover:border-green-300 hover:bg-green-100 transition-all"
        >
          <div className="p-3 rounded-xl bg-green-500 text-white group-hover:scale-110 transition-transform">
            <Unlock className="h-7 w-7" />
          </div>
          <span className="font-semibold text-green-700">Unlock</span>
          <span className="text-xs text-green-400">Resume operation</span>
        </button>

        <button
          onClick={() => sendCommand('ALERT')}
          className="group flex flex-col items-center gap-3 p-6 rounded-2xl bg-yellow-50 border-2 border-yellow-100 hover:border-yellow-300 hover:bg-yellow-100 transition-all"
        >
          <div className="p-3 rounded-xl bg-yellow-500 text-white group-hover:scale-110 transition-transform">
            <Bell className="h-7 w-7" />
          </div>
          <span className="font-semibold text-yellow-700">Alert</span>
          <span className="text-xs text-yellow-500">Trigger alarm</span>
        </button>
      </div>
    </div>
  );
}
