'use client';

import React, { useEffect, useState } from 'react';
import { fetchUsername } from '../utils/fetchUsername';
import { motion } from 'framer-motion';

const HeaderBox: React.FC = () => {
  const [username, setUsername] = useState<string | null>(null);

  // Fetch username on mount
  useEffect(() => {
    const getUsername = async () => {
      const fetchedUsername = await fetchUsername();
      setUsername(fetchedUsername);
    };

    getUsername();
  }, []);

  // Framer Motion animation variants for the text
  const textVariants = {
    hidden: { opacity: 0, y: -20 },
    visible: {
      opacity: 1,
      y: 0,
      transition: { duration: 0.6, ease: 'easeOut' },
    },
  };

  // Subtext animation
  const subTextVariants = {
    hidden: { opacity: 0, y: 10 },
    visible: {
      opacity: 1,
      y: 0,
      transition: { duration: 0.6, ease: 'easeOut', delay: 0.3 },
    },
  };

  return (
    <div className="w-full border-b px-4 py-4 sm:px-6 sm:py-6">
      <motion.div
        initial="hidden"
        animate="visible"
        variants={textVariants}
        className="w-full"
      >
        <h1 className="text-xl font-poppins font-semibold text-gray-800 sm:text-2xl lg:text-3xl">
          Welcome Back,{' '}
          <span className="bg-gradient-to-r from-green-400 to-green-600 bg-clip-text text-transparent font-poppins">
            {username ? `${username}` : 'Guest'}
          </span>
          !
        </h1>
        <motion.p
          initial="hidden"
          animate="visible"
          variants={subTextVariants}
          className="mt-2 text-sm font-normal text-gray-600 sm:text-base"
        >
          Ready to make a difference? Let’s clean up the world together.
        </motion.p>
      </motion.div>
    </div>
  );
};

export default HeaderBox;
