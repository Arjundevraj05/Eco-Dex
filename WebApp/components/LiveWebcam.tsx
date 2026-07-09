'use client';

import React, { useEffect, useState } from 'react';
import {
  SlidersHorizontal,
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  Bot,
  Crosshair,
  Zap,
  Navigation,
} from 'lucide-react';

const directionLabels: Record<string, string> = {
  w: 'Forward',
  a: 'Left',
  s: 'Backward',
  d: 'Right',
};

const LiveWebcam: React.FC = () => {
  const [activeKey, setActiveKey] = useState<string | null>(null);
  const [lastCommand, setLastCommand] = useState<string | null>(null);
  const [commandHistory, setCommandHistory] = useState<string[]>([]);

  const recordCommand = (key: string) => {
    const label = directionLabels[key];
    setActiveKey(key);
    setLastCommand(label);
    setCommandHistory((prev) => [label, ...prev].slice(0, 5));
  };

  const handleKeyPress = (e: KeyboardEvent) => {
    if (['w', 'a', 's', 'd'].includes(e.key)) recordCommand(e.key);
  };

  const handleKeyRelease = () => setActiveKey(null);

  useEffect(() => {
    window.addEventListener('keydown', handleKeyPress);
    window.addEventListener('keyup', handleKeyRelease);
    return () => {
      window.removeEventListener('keydown', handleKeyPress);
      window.removeEventListener('keyup', handleKeyRelease);
    };
  }, []);

  const btnClass = (key: string) =>
    `flex items-center justify-center w-14 h-14 rounded-xl font-bold transition-all shadow-md ${
      activeKey === key
        ? 'bg-green-700 scale-95 ring-2 ring-green-400 text-white'
        : 'bg-gradient-to-br from-green-500 to-green-600 hover:from-green-600 hover:to-green-700 text-white hover:shadow-lg'
    }`;

  return (
    <div className="ml-40 min-h-screen bg-green-50 py-8 px-6">
      <div className="max-w-5xl mx-auto">
        {/* Header */}
        <div className="mb-6 rounded-2xl bg-gradient-to-r from-green-700 to-emerald-600 p-6 text-white shadow-lg">
          <div className="flex items-center gap-3 mb-1">
            <SlidersHorizontal className="h-7 w-7" />
            <span className="text-sm font-medium uppercase tracking-wider opacity-90">Robot Control</span>
          </div>
          <h1 className="text-3xl font-bold">Manual Override</h1>
          <p className="mt-1 text-green-100 text-sm">
            Take direct control of RAG-ED movement — use WASD keys or the control pad below
          </p>
        </div>

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
          {/* Feed */}
          <div className="lg:col-span-2 bg-white rounded-2xl shadow-md overflow-hidden border border-green-100">
            <div className="relative h-80 bg-slate-900">
              <div
                className="absolute inset-0 opacity-20"
                style={{
                  backgroundImage:
                    'linear-gradient(rgba(74,222,128,0.4) 1px, transparent 1px), linear-gradient(90deg, rgba(74,222,128,0.4) 1px, transparent 1px)',
                  backgroundSize: '32px 32px',
                }}
              />
              <div className="absolute inset-0 bg-gradient-to-t from-slate-900/80 via-transparent to-slate-900/30" />

              <div className="absolute top-4 left-4 flex items-center gap-2 bg-black/60 rounded-full px-3 py-1">
                <span className="w-2 h-2 rounded-full bg-red-500 animate-pulse" />
                <span className="text-white text-xs font-medium">SIMULATED FEED</span>
              </div>
              <div className="absolute top-4 right-4 text-green-400 text-xs font-mono">RAG-ED · CAM-01</div>

              <div className="absolute inset-0 flex flex-col items-center justify-center text-center px-8">
                <Bot className="h-14 w-14 text-green-400/80 mb-3" />
                <p className="text-white font-semibold text-lg">Robot POV</p>
                <p className="text-green-300/60 text-sm mt-1">Demo mode — hardware feed unavailable</p>
              </div>

              <Crosshair className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 h-12 w-12 text-green-500/30" />

              <div className="absolute bottom-4 left-4 right-4 flex justify-between text-xs font-mono text-green-400/90">
                <span>SPD: {activeKey ? '1.2 m/s' : '0.0 m/s'}</span>
                <span className="text-white font-semibold">
                  {lastCommand ? `▶ ${lastCommand.toUpperCase()}` : '■ IDLE'}
                </span>
                <span>BAT: 87%</span>
              </div>
            </div>
          </div>

          {/* Telemetry */}
          <div className="space-y-4">
            <div className="bg-white rounded-2xl shadow-md p-5 border border-green-100">
              <h3 className="text-sm font-semibold text-gray-500 uppercase tracking-wide mb-3">Status</h3>
              <div className="space-y-3">
                <TelemetryRow icon={<Zap className="h-4 w-4 text-yellow-500" />} label="Mode" value="Manual" />
                <TelemetryRow icon={<Navigation className="h-4 w-4 text-green-500" />} label="Heading" value={lastCommand || 'Stationary'} />
                <TelemetryRow icon={<Bot className="h-4 w-4 text-blue-500" />} label="Robot" value="RAG-ED #01" />
              </div>
            </div>

            <div className="bg-white rounded-2xl shadow-md p-5 border border-green-100">
              <h3 className="text-sm font-semibold text-gray-500 uppercase tracking-wide mb-3">Recent Commands</h3>
              {commandHistory.length === 0 ? (
                <p className="text-sm text-gray-400 italic">No commands yet</p>
              ) : (
                <ul className="space-y-2">
                  {commandHistory.map((cmd, i) => (
                    <li key={i} className="flex items-center gap-2 text-sm text-gray-700">
                      <span className="w-1.5 h-1.5 rounded-full bg-green-500" />
                      {cmd}
                    </li>
                  ))}
                </ul>
              )}
            </div>
          </div>
        </div>

        {/* D-pad */}
        <div className="mt-6 bg-white rounded-2xl shadow-md p-8 border border-green-100">
          <h2 className="text-center text-lg font-semibold text-gray-800 mb-1">Drive Controls</h2>
          <p className="text-center text-sm text-gray-400 mb-6">WASD keyboard or click the pad</p>
          <div className="flex flex-col items-center gap-2">
            <button onMouseDown={() => recordCommand('w')} onMouseUp={handleKeyRelease} className={btnClass('w')}>
              <ArrowUp className="h-7 w-7" />
            </button>
            <div className="flex gap-2">
              <button onMouseDown={() => recordCommand('a')} onMouseUp={handleKeyRelease} className={btnClass('a')}>
                <ArrowLeft className="h-7 w-7" />
              </button>
              <div className="w-14 h-14 rounded-xl bg-gray-100 flex items-center justify-center text-gray-400 text-xs font-bold">
                BOT
              </div>
              <button onMouseDown={() => recordCommand('d')} onMouseUp={handleKeyRelease} className={btnClass('d')}>
                <ArrowRight className="h-7 w-7" />
              </button>
            </div>
            <button onMouseDown={() => recordCommand('s')} onMouseUp={handleKeyRelease} className={btnClass('s')}>
              <ArrowDown className="h-7 w-7" />
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

function TelemetryRow({ icon, label, value }: { icon: React.ReactNode; label: string; value: string }) {
  return (
    <div className="flex items-center justify-between">
      <div className="flex items-center gap-2 text-sm text-gray-500">
        {icon}
        {label}
      </div>
      <span className="text-sm font-semibold text-gray-800">{value}</span>
    </div>
  );
}

export default LiveWebcam;
