'use client';
import React, { useEffect, useState } from 'react';
import { FaRobot, FaRegChartBar, FaBell, FaArrowLeft, FaArrowRight, FaRecycle, FaTrash } from 'react-icons/fa';

interface WasteRecord {
  Class: 'PLASTIC' | 'METAL' | 'PAPER' | 'CARDBOARD' | 'GLASS';
}

const RightSidebar = () => {
  const [isExpanded, setIsExpanded] = useState(false);
  const [plasticKg, setPlasticKg] = useState(0);
  const [metalKg, setMetalKg] = useState(0);
  const [totalItems, setTotalItems] = useState(0);

  useEffect(() => {
    const fetchStats = async () => {
      try {
        const response = await fetch('/api/users/current', { credentials: 'include' });
        if (!response.ok) return;
        const data: WasteRecord[] = await response.json();
        setTotalItems(data.length);
        setPlasticKg(data.filter((item) => item.Class === 'PLASTIC').length);
        setMetalKg(data.filter((item) => item.Class === 'METAL').length);
      } catch {
        // Keep defaults when unauthenticated or offline
      }
    };
    fetchStats();
  }, []);

  return (
    <div
      className={`hidden md:block fixed inset-y-0 right-0 z-40 transform transition-transform duration-300 ease-in-out bg-white shadow-lg border-l ${
        isExpanded ? 'translate-x-0 w-48' : 'translate-x-0 w-14'
      }`}
    >
      <button
        type="button"
        aria-label={isExpanded ? 'Collapse sidebar' : 'Expand sidebar'}
        className="absolute top-1/2 left-0 -translate-x-1/2 -translate-y-1/2 rounded-full bg-green-500 p-2 shadow-lg transition-colors hover:bg-green-600"
        onClick={() => setIsExpanded(!isExpanded)}
      >
        {isExpanded ? <FaArrowRight className="text-white" /> : <FaArrowLeft className="text-white" />}
      </button>

      <div className={`h-full overflow-y-auto p-3 ${isExpanded ? 'block' : 'hidden'}`}>
        <div className="mb-5 border-b pb-4">
          <h2 className="mb-3 flex items-center font-semibold text-sm text-green-700 font-poppins">
            <FaRobot className="mr-2 shrink-0" /> Bot Status
          </h2>
          <p className="text-sm font-light font-poppins text-gray-700">
            Status: <span className="font-medium text-green-700">Active</span>
          </p>
          <p className="text-sm font-light font-poppins text-gray-700">
            Items collected: <span className="font-medium text-green-700">{totalItems}</span>
          </p>
        </div>

        <div className="mb-5 border-b pb-4">
          <h2 className="mb-3 flex items-center font-semibold text-sm text-green-700 font-poppins">
            <FaTrash className="mr-2 shrink-0" /> Waste Collection
          </h2>
          <p className="text-sm font-light font-poppins text-gray-700">
            Plastic: <span className="font-medium text-green-700">{plasticKg} items</span>
          </p>
          <p className="text-sm font-light font-poppins text-gray-700">
            Metal: <span className="font-medium text-green-700">{metalKg} items</span>
          </p>
        </div>

        <div className="mb-5 border-b pb-4">
          <h2 className="mb-3 flex items-center font-semibold text-sm text-green-700 font-poppins">
            <FaBell className="mr-2 shrink-0" /> Alerts
          </h2>
          <p className="text-sm font-light font-poppins text-gray-700">No alerts at this time.</p>
        </div>

        <div className="mb-5 border-b pb-4">
          <h2 className="mb-3 flex items-center font-semibold text-sm text-green-700 font-poppins">
            <FaRegChartBar className="mr-2 shrink-0" /> Upcoming
          </h2>
          <p className="text-sm font-light font-poppins text-gray-700">
            Next mission: <span className="font-medium text-green-700">Sector A at 3 PM</span>
          </p>
        </div>

        <div className="mb-5 border-b pb-4">
          <h2 className="mb-3 flex items-center font-semibold text-sm text-green-700 font-poppins">
            <FaRecycle className="mr-2 shrink-0" /> Tips
          </h2>
          <p className="text-sm font-light font-poppins text-gray-700">Recycle plastics whenever possible!</p>
        </div>
      </div>

      <div className={`flex h-full flex-col items-center p-3 pt-6 ${!isExpanded ? 'flex' : 'hidden'}`}>
        <FaRobot className="mb-5 text-xl text-green-700" />
        <FaTrash className="mb-5 text-xl text-green-700" />
        <FaBell className="mb-5 text-xl text-green-700" />
        <FaRegChartBar className="mb-5 text-xl text-green-700" />
        <FaRecycle className="mb-5 text-xl text-green-700" />
      </div>
    </div>
  );
};

export default RightSidebar;
