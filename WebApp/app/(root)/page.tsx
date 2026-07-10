"use client";

import { Leaf, Coins, Users } from 'lucide-react';
import HeaderBox from "@/components/HeaderBox";
import Status1 from "@/components/Status1";
import React from "react";

function AnimatedGlobe() {
  return (
    <div className="relative mx-auto mb-6 mt-4 h-24 w-24 sm:mb-8 sm:mt-6 sm:h-32 sm:w-32">
      <div className="absolute inset-0 rounded-full bg-green-500 opacity-20 animate-pulse" />
      <div className="absolute inset-2 rounded-full bg-green-400 opacity-40 animate-ping" />
      <div className="absolute inset-4 rounded-full bg-green-300 opacity-60 animate-spin" />
      <div className="absolute inset-6 rounded-full bg-green-200 opacity-80 animate-bounce" />
      <Leaf className="absolute inset-0 m-auto h-12 w-12 text-green-600 animate-pulse sm:h-16 sm:w-16" />
    </div>
  );
}

const Page: React.FC = () => {
  return (
    <div className="flex min-h-screen w-full flex-col items-center bg-green-50 font-sans">
      <HeaderBox />
      <div className="mb-12 w-full max-w-4xl px-4 pb-12 text-center sm:px-6 md:mb-20">
        <AnimatedGlobe />
        <h1 className="mb-4 text-3xl font-bold tracking-tight text-gray-800 sm:mb-6 sm:text-4xl md:text-5xl lg:text-6xl">
          Revolutionize <br /><span className="text-green-600">Waste Management</span>
        </h1>
        <p className="mx-auto mb-8 max-w-2xl text-base leading-relaxed text-gray-600 sm:text-lg md:text-xl">
          With Eco-Dex and RAG-ED, experience a smarter, greener approach to waste collection!
        </p>

        <div className="mb-10 grid grid-cols-1 gap-6 sm:grid-cols-2 md:grid-cols-3 md:gap-8">
          <FeatureCard
            icon={Leaf}
            title="Eco-Friendly"
            description="Contribute to a cleaner environment by reporting and collecting waste."
          />
          <FeatureCard
            icon={Coins}
            title="Earn Rewards"
            description="Get tokens for your contributions to waste management efforts."
          />
          <FeatureCard
            icon={Users}
            title="Community-Driven"
            description="Be part of a growing community committed to sustainable practices."
          />
        </div>

        <section className="mx-auto w-full max-w-4xl overflow-hidden rounded-2xl bg-white p-6 shadow-lg sm:rounded-3xl sm:p-8 md:p-10 lg:max-w-6xl">
          <h2 className="mb-8 text-center text-2xl font-bold text-gray-800 sm:mb-10 sm:text-3xl md:text-4xl">
            Our Impact
          </h2>
          <Status1 />
        </section>
      </div>
    </div>
  );
};

interface FeatureCardProps {
  icon: React.ElementType;
  title: string;
  description: string;
}

const FeatureCard: React.FC<FeatureCardProps> = ({ icon: Icon, title, description }) => {
  return (
    <div className="flex flex-col items-center rounded-xl bg-white p-6 text-center shadow-md transition-all duration-300 ease-in-out hover:shadow-lg sm:p-8">
      <div className="mb-4 rounded-full bg-green-100 p-3 sm:mb-6 sm:p-4">
        <Icon className="h-8 w-8 text-green-600 sm:h-10 sm:w-10" />
      </div>
      <h3 className="mb-3 text-lg font-semibold text-gray-800 sm:mb-4 sm:text-xl">{title}</h3>
      <p className="text-sm leading-relaxed text-gray-600 sm:text-base">{description}</p>
    </div>
  );
};

export default Page;
