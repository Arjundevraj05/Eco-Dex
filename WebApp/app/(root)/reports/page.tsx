'use client';
import React, { useEffect, useState } from 'react';
import { Doughnut, Line, Bar } from 'react-chartjs-2';
import {
  Chart as ChartJS,
  ArcElement,
  Tooltip,
  Legend,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  BarElement,
  Filler,
} from 'chart.js';
import CountUp from 'react-countup';
import { BarChart3, Leaf, Trash2, TrendingUp } from 'lucide-react';

ChartJS.register(
  ArcElement, Tooltip, Legend, CategoryScale, LinearScale,
  PointElement, LineElement, BarElement, Filler
);

interface WasteRecord {
  isBiodegradable: boolean;
  Class: 'PLASTIC' | 'METAL' | 'PAPER' | 'CARDBOARD' | 'GLASS';
  Day: 'Monday' | 'Tuesday' | 'Wednesday' | 'Thursday' | 'Friday' | 'Saturday' | 'Sunday';
}

const CLASS_COLORS: Record<string, string> = {
  PLASTIC: '#22c55e',
  METAL: '#f59e0b',
  PAPER: '#3b82f6',
  CARDBOARD: '#84cc16',
  GLASS: '#ef4444',
};

const ReportsPage = () => {
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [biodegradableCount, setBiodegradableCount] = useState(0);
  const [totalWasteCount, setTotalWasteCount] = useState(0);
  const [plasticCount, setPlasticCount] = useState(0);
  const [metalCount, setMetalCount] = useState(0);
  const [paperCount, setPaperCount] = useState(0);
  const [cardboardCount, setCardboardCount] = useState(0);
  const [glassCount, setGlassCount] = useState(0);
  const [dayCounts, setDayCounts] = useState({
    Monday: 0, Tuesday: 0, Wednesday: 0, Thursday: 0,
    Friday: 0, Saturday: 0, Sunday: 0,
  });

  useEffect(() => {
    const fetchReports = async () => {
      try {
        const response = await fetch('/api/users/current', { credentials: 'include' });
        if (!response.ok) throw new Error('Failed to fetch records');
        const data: WasteRecord[] = await response.json();
        setTotalWasteCount(data.length);
        setBiodegradableCount(data.filter((item) => item.isBiodegradable).length);
        setPlasticCount(data.filter((item) => item.Class === 'PLASTIC').length);
        setMetalCount(data.filter((item) => item.Class === 'METAL').length);
        setPaperCount(data.filter((item) => item.Class === 'PAPER').length);
        setCardboardCount(data.filter((item) => item.Class === 'CARDBOARD').length);
        setGlassCount(data.filter((item) => item.Class === 'GLASS').length);
        setDayCounts({
          Monday: data.filter((item) => item.Day === 'Monday').length,
          Tuesday: data.filter((item) => item.Day === 'Tuesday').length,
          Wednesday: data.filter((item) => item.Day === 'Wednesday').length,
          Thursday: data.filter((item) => item.Day === 'Thursday').length,
          Friday: data.filter((item) => item.Day === 'Friday').length,
          Saturday: data.filter((item) => item.Day === 'Saturday').length,
          Sunday: data.filter((item) => item.Day === 'Sunday').length,
        });
      } catch (err: unknown) {
        setError(err instanceof Error ? err.message : 'An unexpected error occurred');
      } finally {
        setLoading(false);
      }
    };
    fetchReports();
  }, []);

  if (loading) {
    return (
      <div className="ml-40 min-h-screen bg-green-50 flex items-center justify-center">
        <div className="flex flex-col items-center gap-3">
          <div className="h-10 w-10 rounded-full border-4 border-green-200 border-t-green-600 animate-spin" />
          <p className="text-gray-500">Loading analysis...</p>
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="ml-40 min-h-screen bg-green-50 flex items-center justify-center">
        <p className="text-red-500">Error: {error}</p>
      </div>
    );
  }

  const classBreakdown = [
    { label: 'Plastic', count: plasticCount, color: CLASS_COLORS.PLASTIC },
    { label: 'Metal', count: metalCount, color: CLASS_COLORS.METAL },
    { label: 'Paper', count: paperCount, color: CLASS_COLORS.PAPER },
    { label: 'Cardboard', count: cardboardCount, color: CLASS_COLORS.CARDBOARD },
    { label: 'Glass', count: glassCount, color: CLASS_COLORS.GLASS },
  ];

  const doughnutData = {
    labels: classBreakdown.map((c) => c.label),
    datasets: [{
      data: classBreakdown.map((c) => c.count),
      backgroundColor: classBreakdown.map((c) => c.color),
      borderWidth: 2,
      borderColor: '#fff',
      hoverOffset: 8,
    }],
  };

  const lineData = {
    labels: Object.keys(dayCounts),
    datasets: [{
      label: 'Items Collected',
      data: Object.values(dayCounts),
      fill: true,
      backgroundColor: 'rgba(34, 197, 94, 0.12)',
      borderColor: '#16a34a',
      borderWidth: 2.5,
      tension: 0.4,
      pointBackgroundColor: '#16a34a',
      pointRadius: 5,
      pointHoverRadius: 7,
    }],
  };

  const barData = {
    labels: ['Bio', 'Non-Bio'],
    datasets: [{
      data: [biodegradableCount, totalWasteCount - biodegradableCount],
      backgroundColor: ['#22c55e', '#f87171'],
      borderRadius: 8,
      borderSkipped: false,
    }],
  };

  const chartOptions = {
    plugins: { legend: { display: false } },
    scales: {
      y: { beginAtZero: true, grid: { color: '#f0fdf4' } },
      x: { grid: { display: false } },
    },
  };

  return (
    <div className="ml-40 min-h-screen bg-green-50 py-8 px-6">
      <div className="max-w-5xl mx-auto">
        {/* Header */}
        <div className="mb-8 rounded-2xl bg-gradient-to-r from-green-600 to-emerald-500 p-6 text-white shadow-lg">
          <div className="flex items-center gap-3 mb-2">
            <BarChart3 className="h-7 w-7" />
            <span className="text-sm font-medium uppercase tracking-wider opacity-90">RAG-ED Analytics</span>
          </div>
          <h1 className="text-3xl font-bold">Performance Analysis</h1>
          <p className="mt-1 text-green-100 text-sm">
            Waste collection breakdown, trends, and environmental impact metrics
          </p>
        </div>

        {/* Summary cards */}
        <div className="grid grid-cols-1 sm:grid-cols-3 gap-5 mb-8">
          <SummaryCard
            icon={<Trash2 className="h-6 w-6 text-green-600" />}
            label="Total Collected"
            value={totalWasteCount}
            accent="border-green-500"
          />
          <SummaryCard
            icon={<Leaf className="h-6 w-6 text-emerald-600" />}
            label="Biodegradable"
            value={biodegradableCount}
            accent="border-emerald-500"
          />
          <SummaryCard
            icon={<TrendingUp className="h-6 w-6 text-orange-500" />}
            label="Non-Biodegradable"
            value={totalWasteCount - biodegradableCount}
            accent="border-orange-400"
          />
        </div>

        {/* Charts row */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
          <div className="bg-white rounded-2xl shadow-md p-6 border border-green-100">
            <h2 className="text-lg font-semibold text-gray-800 mb-1">Waste Composition</h2>
            <p className="text-xs text-gray-400 mb-4">By material type</p>
            <div className="max-w-[220px] mx-auto">
              <Doughnut
                data={doughnutData}
                options={{ plugins: { legend: { position: 'bottom', labels: { boxWidth: 12, padding: 16 } } } }}
              />
            </div>
          </div>

          <div className="bg-white rounded-2xl shadow-md p-6 border border-green-100">
            <h2 className="text-lg font-semibold text-gray-800 mb-1">Weekly Trends</h2>
            <p className="text-xs text-gray-400 mb-4">Collections per day</p>
            <div className="h-56">
              <Line data={lineData} options={{ ...chartOptions, maintainAspectRatio: false }} />
            </div>
          </div>
        </div>

        {/* Bottom row */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
          <div className="bg-white rounded-2xl shadow-md p-6 border border-green-100">
            <h2 className="text-lg font-semibold text-gray-800 mb-4">Material Breakdown</h2>
            <div className="space-y-3">
              {classBreakdown.map(({ label, count, color }) => (
                <div key={label}>
                  <div className="flex justify-between text-sm mb-1">
                    <span className="text-gray-600">{label}</span>
                    <span className="font-semibold text-gray-800">{count}</span>
                  </div>
                  <div className="h-2.5 bg-gray-100 rounded-full overflow-hidden">
                    <div
                      className="h-full rounded-full transition-all duration-700"
                      style={{
                        width: totalWasteCount ? `${(count / totalWasteCount) * 100}%` : '0%',
                        backgroundColor: color,
                      }}
                    />
                  </div>
                </div>
              ))}
            </div>
          </div>

          <div className="bg-white rounded-2xl shadow-md p-6 border border-green-100">
            <h2 className="text-lg font-semibold text-gray-800 mb-1">Biodegradability Split</h2>
            <p className="text-xs text-gray-400 mb-4">Environmental classification</p>
            <div className="h-48">
              <Bar data={barData} options={{ ...chartOptions, maintainAspectRatio: false }} />
            </div>
            <div className="flex justify-center gap-6 mt-2 text-sm">
              <span className="flex items-center gap-1.5">
                <span className="w-3 h-3 rounded-full bg-green-500" />
                Bio ({biodegradableCount})
              </span>
              <span className="flex items-center gap-1.5">
                <span className="w-3 h-3 rounded-full bg-red-400" />
                Non-Bio ({totalWasteCount - biodegradableCount})
              </span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

function SummaryCard({
  icon, label, value, accent,
}: {
  icon: React.ReactNode;
  label: string;
  value: number;
  accent: string;
}) {
  return (
    <div className={`bg-white rounded-2xl shadow-md p-5 border-l-4 ${accent} hover:shadow-lg transition-shadow`}>
      <div className="flex items-center gap-3 mb-3">
        <div className="p-2 bg-green-50 rounded-lg">{icon}</div>
        <p className="text-sm text-gray-500 font-medium">{label}</p>
      </div>
      <p className="text-4xl font-bold text-gray-800">
        <CountUp end={value} duration={2} />
      </p>
    </div>
  );
}

export default ReportsPage;
