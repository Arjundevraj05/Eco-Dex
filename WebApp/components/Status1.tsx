"use client";
import React, { useEffect, useState } from "react";
import Image from "next/image";
import CountUp from "react-countup";
import { Coins, Leaf } from "lucide-react";

interface WasteItem {
  isBiodegradable: boolean;
  Class: "PLASTIC" | "METAL" | "PAPER" | "CARDBOARD" | "GLASS";
}

const Status1 = () => {
  const [biodegradableCount, setBiodegradableCount] = useState(0);
  const [totalWasteCount, setTotalWasteCount] = useState(0);
  const [tokensEarned, setTokensEarned] = useState(0);
  const [carbonReduced, setCarbonReduced] = useState(0);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const fetchReports = async () => {
      try {
        const response = await fetch("/api/users/current", { credentials: "include" });
        if (!response.ok) {
          throw new Error("Failed to fetch records");
        }
        const data: WasteItem[] = await response.json();

        setTotalWasteCount(data.length);

        const biodegradableItems = data.filter((item) => item.isBiodegradable);
        setBiodegradableCount(biodegradableItems.length);

        const counts = data.reduce(
          (acc, item) => {
            acc[item.Class] = (acc[item.Class] || 0) + 1;
            return acc;
          },
          { PLASTIC: 0, METAL: 0, PAPER: 0, CARDBOARD: 0, GLASS: 0 }
        );

        // Calculate Tokens Earned & Carbon Emissions Reduced
        setTokensEarned(
          counts.PLASTIC * 2 + counts.METAL * 3 + counts.PAPER * 1 + counts.CARDBOARD * 2 + counts.GLASS * 2
        );

        setCarbonReduced(
          counts.PLASTIC * 6 + counts.METAL * 10 + counts.PAPER * 4 + counts.CARDBOARD * 5 + counts.GLASS * 3
        );
      } catch (err) {
        console.error("Error fetching data:", err);
      } finally {
        setLoading(false);
      }
    };

    fetchReports();
  }, []);

  if (loading) {
    return (
      <div className="mb-6 mt-10 flex justify-center">
        <p className="text-gray-500">Loading impact stats...</p>
      </div>
    );
  }

  return (
    <div className="mb-6 mt-10 flex justify-center flex-row gap-3 flex-wrap">
      {/* Biodegradable Section */}
      <div className="cursor-pointer flex flex-row items-center bg-gradient-to-b from-green-300 to-green-500 rounded-lg p-[2px] hover:scale-105 transition-transform duration-200 w-72 h-36">
        <div className="flex items-center bg-gray-50 rounded-lg p-5 w-full h-full">
          <div className="rounded-full p-3 bg-gradient-to-b from-green-100 to-green-200 flex-shrink-0">
            <Image src="/icons/biodegradable.svg" width={50} height={50} alt="Biodegradable Icon" />
          </div>
          <div className="ml-4 flex-grow">
            <p className="font-semibold text-xl text-green-800">Biodegradable</p>
            <p className="font-bold text-3xl mt-2 text-green-900">
              <CountUp end={biodegradableCount} duration={2.5} />
            </p>
          </div>
        </div>
      </div>

      {/* Non-Biodegradable Section */}
      <div className="cursor-pointer flex flex-row items-center bg-gradient-to-b from-red-300 to-red-500 rounded-lg p-[2px] hover:scale-105 transition-transform duration-200 w-72 h-36">
        <div className="flex items-center bg-gray-50 rounded-lg p-5 w-full h-full">
          <div className="rounded-full p-3 bg-gradient-to-b from-red-100 to-red-200 flex-shrink-0">
            <Image src="/icons/nonbiodegradable.svg" width={50} height={50} alt="Non-Biodegradable Icon" />
          </div>
          <div className="ml-4 flex-grow">
            <p className="font-semibold text-xl text-red-800">Non-Biodegradable</p>
            <p className="font-bold text-3xl mt-2 text-red-900">
              <CountUp end={totalWasteCount - biodegradableCount} duration={2.5} />
            </p>
          </div>
        </div>
      </div>

      {/* Carbon Emissions Reduced Section */}
      <div className="cursor-pointer flex flex-row items-center bg-gradient-to-b from-gray-300 to-gray-500 rounded-lg p-[2px] hover:scale-105 transition-transform duration-200 w-72 h-36">
        <div className="flex items-center bg-gray-50 rounded-lg p-5 w-full h-full">
          <div className="rounded-full p-3 bg-gradient-to-b from-gray-100 to-gray-200 flex-shrink-0">
            <Leaf className="h-12 w-12 text-gray-700" />
          </div>
          <div className="ml-4 flex-grow">
            <p className="font-semibold text-xl text-gray-700">Carbon Reduced</p>
            <p className="font-bold text-3xl mt-2 text-gray-800">
              <CountUp end={carbonReduced} duration={2.5} /> kg
            </p>
          </div>
        </div>
      </div>

      {/* Tokens Earned Section */}
      <div className="cursor-pointer flex flex-row items-center bg-gradient-to-b from-yellow-300 to-yellow-500 rounded-lg p-[2px] hover:scale-105 transition-transform duration-200 w-72 h-36">
        <div className="flex items-center bg-gray-50 rounded-lg p-5 w-full h-full">
          <div className="rounded-full p-3 bg-gradient-to-b from-yellow-100 to-yellow-200 flex-shrink-0">
            <Coins className="h-12 w-12 text-yellow-600" />
          </div>
          <div className="ml-4 flex-grow">
            <p className="font-semibold text-xl text-yellow-800">Tokens Earned</p>
            <p className="font-bold text-3xl mt-2 text-yellow-900">
              <CountUp end={tokensEarned} duration={2.5} />
            </p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default Status1;