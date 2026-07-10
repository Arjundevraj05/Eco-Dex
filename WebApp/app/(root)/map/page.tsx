"use client"
import dynamic from 'next/dynamic';
import React, { useEffect, useState } from 'react';

const MapComponent = dynamic(() => import('@/components/MapComponent'), {
  ssr: false,
  loading: () => (
    <div className="flex h-full w-full items-center justify-center rounded-lg bg-gray-100 text-sm text-gray-500">
      Loading map...
    </div>
  ),
});

interface WasteRecord {
  Latitude: number;
  Longitude: number;
}

const Page = () => {
  const [locations, setLocations] = useState<{ lat: number; lng: number }[]>([]);

  useEffect(() => {
    const fetchWasteRecords = async () => {
      try {
        const response = await fetch('/api/users/current', { credentials: 'include' });
        if (!response.ok) {
          throw new Error('Failed to fetch records');
        }
        const data: WasteRecord[] = await response.json();

        const filteredLocations = data
          .filter((item) => item.Latitude !== undefined && item.Longitude !== undefined)
          .map((item) => ({
            lat: item.Latitude,
            lng: item.Longitude,
          }));

        setLocations(filteredLocations);
      } catch (err) {
        console.error('Error fetching waste records:', err);
      }
    };

    fetchWasteRecords();
  }, []);

  return (
    <div className="flex min-h-screen flex-col items-center bg-green-50 p-4 font-sans sm:p-6">
      <div className="w-full max-w-4xl rounded-lg bg-white p-4 shadow-xl sm:p-6">
        <div className="mb-5 text-center sm:mb-7">
          <h2 className="text-2xl font-bold tracking-wide text-gray-800 sm:text-3xl md:text-4xl">
            <span className="font-stacker font-bold">RAG-ED Location</span>
          </h2>
        </div>
        <div className="relative z-0 isolate h-[50vh] overflow-hidden rounded-lg sm:h-[60vh] md:h-[70vh]">
          <MapComponent locations={locations} />
        </div>
      </div>
    </div>
  );
};

export default Page;
