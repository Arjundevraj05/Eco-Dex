"use client"
import MapComponent from '@/components/MapComponent';
import React, { useEffect, useState } from 'react';

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
    <div className='flex flex-col items-center min-h-screen bg-green-50 font-sans p-6'>
      <div className="ml-40 w-full max-w-4xl bg-white shadow-xl rounded-lg p-6">
        <div className="mb-7 text-center">
          <h2 className="text-4xl font-bold text-gray-800 tracking-wide">
            <span className="font-stacker font-bold">RAG-ED Location</span>
          </h2>
        </div>
        <div className="h-[70vh] overflow-hidden">
          <MapComponent locations={locations} />
        </div>
      </div>
    </div>
  );
};

export default Page;
